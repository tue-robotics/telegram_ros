import asyncio
import functools
import threading

import cv2
import numpy as np
import rospy

from io import BytesIO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import String, Header
from telegram import Location, ReplyKeyboardMarkup, Update
from telegram.error import TimedOut
from telegram.ext import ApplicationBuilder, CallbackContext, CommandHandler, MessageHandler, filters
from telegram_ros.msg import Options

WHITELIST = "~whitelist"


def telegram_callback(callback_function):
    """
    Decorator to restrict telegram methods only to the active chat or tell to /start one if needed

    :param callback_function: A coroutine callback function taking a telegram.Update and a telegram.ext.CallbackContext
    :return: Wrapped coroutine callback function
    """

    @functools.wraps(callback_function)
    async def wrapper(self, update: Update, context: CallbackContext):
        rospy.logdebug("Incoming update from telegram: %s", update)
        if self._telegram_chat_id is None:
            rospy.logwarn("Discarding message. No active chat_id.")
            await update.message.reply_text("ROS Bridge not initialized. Type /start to set-up ROS bridge")
        elif self._telegram_chat_id != update.message.chat_id:
            rospy.logwarn("Discarding message. Invalid chat_id")
            await update.message.reply_text(
                "ROS Bridge initialized to another chat_id. Type /start to connect to this chat_id"
            )
        else:
            await callback_function(self, update, context)

    return wrapper


def ros_callback(callback_function):
    """
    Decorator that verifies whether we have an active chat id and handles possible exceptions

    :param callback_function: A callback function taking a ros msg
    :return: Wrapped callback function
    """

    @functools.wraps(callback_function)
    def wrapper(self, msg):
        if not self._telegram_chat_id:
            rospy.logerr("ROS Bridge not initialized, dropping message of type %s", msg._type)
        else:
            try:
                callback_function(self, msg)
            except TimedOut as e:
                rospy.logerr("Telegram timeout: %s", e)

    return wrapper


class TelegramROSBridge:
    def __init__(self, api_token, caption_as_frame_id):
        """
        Telegram ROS bridge that bridges between a Telegram chat conversation and ROS

        :param api_token: The Telegram API token
        """
        self._caption_as_frame_id = caption_as_frame_id

        self._from_telegram_string_publisher = rospy.Publisher("message_to_ros", String, queue_size=10)
        self._from_telegram_image_publisher = rospy.Publisher("image_to_ros", Image, queue_size=10)
        self._from_telegram_location_publisher = rospy.Publisher("location_to_ros", NavSatFix, queue_size=10)

        self._to_telegram_string_subscriber = rospy.Subscriber(
            "message_from_ros", String, self._ros_string_callback, queue_size=10
        )
        self._to_telegram_image_subscriber = rospy.Subscriber(
            "image_from_ros", Image, self._ros_image_callback, queue_size=10
        )
        self._to_telegram_location_subscriber = rospy.Subscriber(
            "location_from_ros", NavSatFix, self._ros_location_callback, queue_size=10
        )
        self._to_telegram_options_subscriber = rospy.Subscriber(
            "options_from_ros", Options, self._ros_options_callback, queue_size=10
        )

        # Set CvBridge
        self._cv_bridge = CvBridge()

        # Telegram IO
        self._telegram_chat_id = None

        # As of python-telegram-bot 20, the library is fully asyncio based, while rospy is synchronous and
        # rospy.spin() blocks the main thread. We therefore run the Telegram Application (together with its own
        # asyncio event loop) in a dedicated background thread. Coroutines triggered from rospy (subscriber) threads
        # are scheduled onto this loop with asyncio.run_coroutine_threadsafe.
        self._event_loop = asyncio.new_event_loop()
        self._loop_running = threading.Event()
        self._stop_event = None  # asyncio.Event, created inside the event loop thread
        self._telegram_thread = threading.Thread(target=self._run_event_loop, name="telegram_event_loop", daemon=True)

        self._application = ApplicationBuilder().token(api_token).build()
        self._application.add_error_handler(self._telegram_error_callback)
        self._application.add_handler(CommandHandler("start", self._telegram_start_callback))
        self._application.add_handler(CommandHandler("stop", self._telegram_stop_callback))
        self._application.add_handler(MessageHandler(filters.TEXT, self._telegram_message_callback))
        self._application.add_handler(MessageHandler(filters.PHOTO, self._telegram_photo_callback))
        self._application.add_handler(MessageHandler(filters.LOCATION, self._telegram_location_callback))

        rospy.core.add_preshutdown_hook(self._shutdown)

    def _run_event_loop(self):
        """
        Thread target: install the event loop on this thread and run the Telegram Application until shutdown.
        """
        asyncio.set_event_loop(self._event_loop)
        try:
            self._event_loop.run_until_complete(self._telegram_main())
        finally:
            self._event_loop.close()

    async def _telegram_main(self):
        """
        Initialize, start and poll the Telegram Application, then wait for the stop event before shutting down.
        """
        self._stop_event = asyncio.Event()
        await self._application.initialize()
        await self._application.start()
        await self._application.updater.start_polling()
        self._loop_running.set()
        try:
            await self._stop_event.wait()
        finally:
            if self._application.updater.running:
                await self._application.updater.stop()
            if self._application.running:
                await self._application.stop()
            await self._application.shutdown()

    def _run_coroutine(self, coroutine):
        """
        Schedule a coroutine on the Telegram event loop from another (e.g. rospy) thread and wait for its result.

        :param coroutine: The awaitable to run on the Telegram event loop
        :return: The result of the coroutine
        """
        future = asyncio.run_coroutine_threadsafe(coroutine, self._event_loop)
        return future.result()

    def _shutdown(self, reason: str):
        """
        Sending a message to the current chat id on destruction.
        """
        if self._telegram_chat_id:
            self._run_coroutine(
                self._application.bot.send_message(
                    self._telegram_chat_id,
                    f"Stopping Telegram ROS bridge, ending this chat. Reason of shutdown: {reason}."
                    " Type /start to connect again after starting a new Telegram ROS bridge.",
                )
            )

    def spin(self):
        """
        Starts the Telegram event loop in a background thread and spins until a SIGINT is received
        """
        self._telegram_thread.start()
        if not self._loop_running.wait(timeout=30.0):
            raise RuntimeError("Telegram event loop failed to start within 30 seconds")
        rospy.loginfo("Telegram updater started polling, spinning ..")

        rospy.spin()
        rospy.loginfo("Shutting down Telegram updater ...")

        self._event_loop.call_soon_threadsafe(self._stop_event.set)
        self._telegram_thread.join(timeout=30.0)

    async def _telegram_error_callback(self, update: object, context: CallbackContext):
        """
        Called when an error occurs while handling a Telegram update.

        :param update: The update that caused the error (may be None)
        :param context: The context holding the error in context.error
        """
        rospy.logerr("Update %s caused error %s", update, context.error)

    async def _telegram_start_callback(self, update: Update, _: CallbackContext):
        """
        Called when a Telegram user sends the '/start' event to the bot, using this event, the bridge can be connected
        to a specific conversation.
        This checks whether the user is on the whitelist if the whitelist is a list and allows any user if whitelist is empty

        :param update: Received update event that holds the chat_id and message data
        """

        # Whitelist defaults to an empty list to disable it and allow anyone. To enable the whitelist, set it to a list with at least one item
        whitelist = rospy.get_param(WHITELIST, [])
        if not whitelist or update.message.from_user.id in whitelist:
            if self._telegram_chat_id is not None and self._telegram_chat_id != update.message.chat_id:
                rospy.logwarn("Changing to different chat_id!")
                new_user = "'somebody'"
                if hasattr(update.message.chat, "first_name") and update.message.chat.first_name:
                    new_user = update.message.chat.first_name
                await self._application.bot.send_message(
                    self._telegram_chat_id,
                    "Lost ROS bridge connection to this chat_id {} ({} took over)".format(
                        update.message.chat_id, new_user
                    ),
                )

            rospy.loginfo("Starting Telegram ROS bridge for new chat id {}".format(update.message.chat_id))
            self._telegram_chat_id = update.message.chat_id

            await update.message.reply_text(
                "Telegram ROS bridge initialized, only replying to chat_id {} (current)".format(self._telegram_chat_id)
            )
        else:
            rospy.logwarn("Discarding message. User {} not whitelisted".format(update.message.from_user))
            await update.message.reply_text(
                "You (user id {}) are not authorized to chat with this bot".format(update.message.from_user.id)
            )

    @telegram_callback
    async def _telegram_stop_callback(self, update: Update, _: CallbackContext):
        """
        Called when a Telegram user sends the '/stop' event to the bot. Then, the user is disconnected from the bot and
        will no longer receive messages.

        :param update: Received update event that holds the chat_id and message data
        """

        rospy.loginfo("Stopping Telegram ROS bridge for chat id {}".format(self._telegram_chat_id))
        await update.message.reply_text(
            "Disconnecting chat_id {}. So long and thanks for all the fish!"
            " Type /start to reconnect".format(self._telegram_chat_id)
        )
        self._telegram_chat_id = None

    @telegram_callback
    async def _telegram_message_callback(self, update: Update, _: CallbackContext):
        """
        Called when a new Telegram message has been received. The method will verify whether the incoming message is
        from the bridges Telegram conversation by comparing the chat_id.

        :param update: Received update that holds the chat_id and message data
        """
        text = update.message.text
        self._from_telegram_string_publisher.publish(String(data=text))

    @ros_callback
    def _ros_string_callback(self, msg: String):
        """
        Called when a new ROS String message is coming in that should be sent to the Telegram conversation

        :param msg: String message
        """
        if msg.data:
            self._run_coroutine(self._application.bot.send_message(self._telegram_chat_id, msg.data))
        else:
            rospy.logwarn("Ignoring empty string message")

    @telegram_callback
    async def _telegram_photo_callback(self, update: Update, _: CallbackContext):
        """
        Called when a new Telegram photo has been received. The method will verify whether the incoming message is
        from the bridges Telegram conversation by comparing the chat_id.

        :param update: Received update that holds the chat_id and message data
        """
        rospy.logdebug("Received image, downloading highest resolution image ...")
        photo_file = await update.message.photo[-1].get_file()
        byte_array = await photo_file.download_as_bytearray()
        rospy.logdebug("Download complete, publishing ...")

        img = cv2.imdecode(np.asarray(byte_array, dtype=np.uint8), cv2.IMREAD_COLOR)
        msg = self._cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")
        msg.header.stamp = rospy.Time.now()

        if self._caption_as_frame_id:
            msg.header.frame_id = update.message.caption
        self._from_telegram_image_publisher.publish(msg)

        if update.message.caption:
            self._from_telegram_string_publisher.publish(String(data=update.message.caption))

    @ros_callback
    def _ros_image_callback(self, msg: Image):
        """
        Called when a new ROS Image message is coming in that should be sent to the Telegram conversation

        :param msg: Image message
        """
        cv2_img = self._cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        self._run_coroutine(
            self._application.bot.send_photo(
                self._telegram_chat_id,
                photo=BytesIO(cv2.imencode(".jpg", cv2_img)[1].tobytes()),
                caption=msg.header.frame_id,
            )
        )

    @telegram_callback
    async def _telegram_location_callback(self, update: Update, _: CallbackContext):
        """
        Called when a new Telegram Location is received. The method will verify whether the incoming Location is
        from the bridged Telegram conversation by comparing the chat_id.

        :param update: Received update that holds the chat_id and message data
        """
        self._from_telegram_location_publisher.publish(
            NavSatFix(
                header=Header(stamp=rospy.Time.now()),
                latitude=update.message.location.latitude,
                longitude=update.message.location.longitude,
                position_covariance_type=NavSatFix.COVARIANCE_TYPE_UNKNOWN,
            )
        )

    @ros_callback
    def _ros_location_callback(self, msg: NavSatFix):
        """
        Called when a new ROS NavSatFix message is coming in that should be sent to the Telegram conversation

        :param msg: NavSatFix that the robot wants to share
        """
        self._run_coroutine(
            self._application.bot.send_location(self._telegram_chat_id, location=Location(msg.longitude, msg.latitude))
        )

    @ros_callback
    def _ros_options_callback(self, msg: Options):
        """
        Called when a new ROS Options message is coming in that should be sent to the Telegram conversation

        :param msg: Options that the robot wants to share
        """

        def chunks(l, n):  # noqa: E741
            """Yield successive n-sized chunks from l."""
            for i in range(0, len(l), n):
                yield l[i : i + n]  # noqa: E203

        self._run_coroutine(
            self._application.bot.send_message(
                self._telegram_chat_id,
                text=msg.question,
                reply_markup=ReplyKeyboardMarkup(
                    list(chunks(msg.options, 5)), resize_keyboard=True, one_time_keyboard=True
                ),
            )
        )

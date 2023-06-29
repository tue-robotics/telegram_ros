import asyncio

import functools

import cv2
import numpy as np
import rospy

from io import BytesIO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import String, Header
from telegram import Location, ReplyKeyboardMarkup, Update
from telegram.error import TimedOut
from telegram.ext import Application, CallbackContext, CommandHandler, MessageHandler, filters
from telegram_ros_msgs.msg import Options


WHITELIST = "~whitelist"


def telegram_callback(callback_function):
    """
    Decorator to restrict telegram methods only to the active chat or tell to /start one if needed

    :param callback_function: A callback function taking a telegram.Bot and a telegram.Update
    :return: Wrapped callback function
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
                asyncio.run(callback_function(self, msg))
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
        self._telegram_app = Application.builder().token(api_token).build()
        self._telegram_app.add_error_handler(
            lambda _, update, error: rospy.logerr("Update {} caused error {}".format(update, error))
        )

        self._telegram_app.add_handler(CommandHandler("start", self._telegram_start_callback))
        self._telegram_app.add_handler(CommandHandler("stop", self._telegram_stop_callback))
        self._telegram_app.add_handler(MessageHandler(filters.TEXT, self._telegram_message_callback))
        self._telegram_app.add_handler(MessageHandler(filters.PHOTO, self._telegram_photo_callback))
        self._telegram_app.add_handler(MessageHandler(filters.LOCATION, self._telegram_location_callback))

        rospy.core.add_preshutdown_hook(self._shutdown)

    def _shutdown(self, reason: str):
        """
        Sending a message to the current chat id on destruction.
        """
        if self._telegram_chat_id:
            asyncio.run(
                self._telegram_app.bot.send_message(
                    self._telegram_chat_id,
                    f"Stopping Telegram ROS bridge, ending this chat. Reason of shutdown: {reason}."
                    " Type /start to connect again after starting a new Telegram ROS bridge.",
                )
            )

    def spin(self):
        """
        Starts the Telegram update thread and spins until a SIGINT is received
        """
        self._telegram_app.run_polling()  # ToDo: this is blocking
        rospy.loginfo("Telegram updater started polling, spinning ..")

        rospy.spin()
        rospy.loginfo("Shutting down Telegram updater ...")

        self._telegram_app.stop()

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
                self._telegram_app.bot.send_message(
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
            f"Disconnecting chat_id {self._telegram_chat_id}. So long and thanks for all the fish!"
            " Type /start to reconnect"
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
    async def _ros_string_callback(self, msg: String):
        """
        Called when a new ROS String message is coming in that should be sent to the Telegram conversation

        :param msg: String message
        """
        if msg.data:
            await self._telegram_app.bot.send_message(self._telegram_chat_id, msg.data)
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
        new_file = await update.message.photo[-1].get_file()
        byte_array = await new_file.download_as_bytearray()
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
    async def _ros_image_callback(self, msg: Image):
        """
        Called when a new ROS Image message is coming in that should be sent to the Telegram conversation

        :param msg: Image message
        """
        cv2_img = self._cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        await self._telegram_app.bot.send_photo(
            self._telegram_chat_id,
            photo=BytesIO(cv2.imencode(".jpg", cv2_img)[1].tobytes()),
            caption=msg.header.frame_id,
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
    async def _ros_location_callback(self, msg: NavSatFix):
        """
        Called when a new ROS NavSatFix message is coming in that should be sent to the Telegram conversation

        :param msg: NavSatFix that the robot wants to share
        """
        await self._telegram_app.bot.send_location(
            self._telegram_chat_id, location=Location(msg.longitude, msg.latitude)
        )

    @ros_callback
    async def _ros_options_callback(self, msg: Options):
        """
        Called when a new ROS Options message is coming in that should be sent to the Telegram conversation

        :param msg: Options that the robot wants to share
        """

        def chunks(l, n):  # noqa: E741
            """Yield successive n-sized chunks from l."""
            for i in range(0, len(l), n):
                yield l[i : i + n]  # noqa: E203

        await self._telegram_app.bot.send_message(
            self._telegram_chat_id,
            text=msg.question,
            reply_markup=ReplyKeyboardMarkup(
                list(chunks(msg.options, 5)), resize_keyboard=True, one_time_keyboard=True
            ),
        )

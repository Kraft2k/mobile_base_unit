import rclpy
from rclpy.node import Node

from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf_transformations
import traceback
from example_interfaces.msg import Float32
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from aruco_interfaces.msg import ArucoMarkers

from geometry_msgs.msg import Twist
import sys
import time
import math
import asyncio
import threading
import telepot
from telepot.loop import MessageLoop
from telepot.namedtuple import InlineKeyboardMarkup, InlineKeyboardButton
import subprocess
from telegram import Update, InlineKeyboardButton, InlineKeyboardMarkup
from telegram.ext import Application, CommandHandler, CallbackQueryHandler, ContextTypes
from telepot.namedtuple import ReplyKeyboardMarkup, KeyboardButton
AUTHORIZED_USERS = [1745570526]


# ros2 run mobile_base_unit mobile_base




# def send_command_to_robot(command: str):
#     # Здесь напишите код для отправки команды роботу
#     print(f"Команда роботу: {command}")
#     node.action(command)

# # Функция для начала диалога с ботом
# async def start(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
#     user_id = update.message.from_user.id
#     if user_id not in AUTHORIZED_USERS:
#         await update.message.reply_text('Извините, у вас нет прав на управление.')
#         return

#     keyboard = [
#         [InlineKeyboardButton("Вперед", callback_data='forward')],
#         [InlineKeyboardButton("Назад", callback_data='backward')],
#         [InlineKeyboardButton("Влево", callback_data='left')],
#         [InlineKeyboardButton("Вправо", callback_data='right')]
#     ]

#     reply_markup = InlineKeyboardMarkup(keyboard)

#     await update.message.reply_text('Выберите команду для управления:', reply_markup=reply_markup)

# # Функция для обработки нажатий на кнопки
# async def button(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
#     query = update.callback_query
#     user_id = query.from_user.id
#     if user_id not in AUTHORIZED_USERS:
#         await query.answer(text='Ошибка', show_alert=True)
#         return

#     await query.answer()

#     command = query.data
#     send_command_to_robot(command)

#     await query.edit_message_text(text=f"Команда: {command}")

# async def run_telegram_bot():
    
#     # Замените 'YOUR_TOKEN' на токен вашего бота
#     application = Application.builder().token("7296965585:AAFGc5lnwy3p2BxnjJiC6aUGP5z5wGqNK6Q").build()

#     application.add_handler(CommandHandler('start', start))
#     application.add_handler(CallbackQueryHandler(button))

#     await application.initialize()
#     await application.start()
#     await application.updater.start_polling()
#     await application.updater.stop()

#     print('БОТ')

# def run_bot():
#     loop = asyncio.new_event_loop()
#     asyncio.set_event_loop(loop)
#     loop.run_until_complete(run_telegram_bot())

# def main2():
#     # Запуск Telegram-бота в отдельном потоке
#     bot_thread = threading.Thread(target=run_bot)
#     bot_thread.start()




import requests
API_TOKEN = "7296965585:AAFGc5lnwy3p2BxnjJiC6aUGP5z5wGqNK6Q"
GROUP_CHAT_ID = '1745570526'
def sendBot(message:str):
    response = requests.get(f'https://api.telegram.org/bot{API_TOKEN}/sendMessage?chat_id={GROUP_CHAT_ID}&text={message}')

def run_command(command):
    try:
        result = subprocess.run(command, shell=True, capture_output=True, text=True)
        print("Output:")
        print(result.stdout)
        if result.stderr:
            print("Errors:")
            print(result.stderr)
    except Exception as e:
        print(f"An error occurred: {e}")

# Функция для отправки команд роботу
def send_command_to_robot(command: str):
    # Здесь напишите код для отправки команды роботу
    print(f"Команда роботу: {command}")
    # Пример отправки команды
    node.action(command)

# Функция для обработки сообщений
def handle(msg):
    content_type, chat_type, chat_id = telepot.glance(msg)
    if chat_id not in AUTHORIZED_USERS:
        bot.sendMessage(chat_id, 'Извините, у вас нет прав на управление.')
        print(f"Пользователь {chat_id} не авторизован")
        return

    if content_type == 'text':
        command = msg['text']

        if command == '/start':
            print(f"Получена команда /start от пользователя {chat_id}")
            keyboard = ReplyKeyboardMarkup(keyboard=[
                [KeyboardButton(text='Вперед')],
                [KeyboardButton(text='Назад')],
                [KeyboardButton(text='Влево'), KeyboardButton(text='Вправо')]
            ], resize_keyboard=True, one_time_keyboard=False)
            bot.sendMessage(chat_id, 'Выберите команду для управления:', reply_markup=keyboard)
            print(f"Пользователю {chat_id} отправлено сообщение с кнопками")
        elif command.startswith('/set speed'):
            try:
                node.speed = float(command[11:])
                bot.sendMessage(chat_id, f"Установленна скорость: {node.speed}")
            except Exception as e:
                bot.sendMessage(chat_id, f"Не коректное значение скорости: {command},{e}")
        elif command.startswith('/console '):
            run_command(command[9:])
        elif command in ('Вперед','Назад','Влево','Вправо'):
            send_command_to_robot(command)


# Функция для обработки нажатий на кнопки
def on_callback_query(msg):
    query_id, chat_id, query_data = telepot.glance(msg, flavor='callback_query')
    if chat_id not in AUTHORIZED_USERS:
        bot.answerCallbackQuery(query_id, text='Ошибка', show_alert=True)
        print(f"Пользователь {chat_id} не авторизован")
        return

    command = query_data
    send_command_to_robot(command)
    #bot.sendMessage(chat_id, f"Команда: {command}")
    print(f"Команда {command} отправлена роботу от пользователя {chat_id}")

# Создаем бота и запускаем его в отдельном потоке
bot = telepot.Bot(API_TOKEN)
MessageLoop(bot, {'chat': handle, 'callback_query': on_callback_query}).run_as_thread()


class FollowArucoMarker(Node):
    def __init__(self):
        super().__init__('follow_aruco_marker')
        self.get_logger().info("Starting follow aruco marker behavior!")
        self.aruco_pose = None
        self.rpm_wheel = 0.0


        self.speed = 0.1

        self.markers_found = []
        self.number_of_the_marker_we_are_going_to = None
        
        self.cmd_pub = self.create_publisher(
            Twist, 'cmd_vel', 10)

        self.subscription_markers = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.markers_callback,
            10)


        self.subscription_rpm = self.create_subscription(
            Float32,
            'left_wheel_rpm',
            self.rpm_callback,
            2)
        
       
        # self.max_distance = 0.7
        self.detect_marker = False
        self.rotation_span = 2.7
        self.waiting_span = 0.3
        self.min_distance = 0.95
        self.distance_to_marker = 0.0
        
        self.lin_max_speed = 0.17
        self.rot_max_speed = 0.013
    def emergency_shutdown(self):
            self.get_logger().warn("Following aruco marker emergency shutdown! Spamming a Twist of 0s!")
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            for i in range(4):
                self.cmd_pub.publish(twist)
                time.sleep(0.05)

    def markers_callback(self, msg:'ArucoMarkers'):
        #print(f'markers_callback')
        self.aruco_pose = msg
    def rpm_callback(self, msg):
        #print(f'rpm_callback')
        self.rpm_wheel = msg
    def action(self, action:str):
        
        forward = 0.0
        rotation = 0.0


        if action == 'Вперед':
            forward = self.speed
        elif action == 'Назад':
            forward = -self.speed
        elif action == 'Влево':
            rotation = 0.1
        elif action == 'Вправо':
            rotation = -0.1



        twist = Twist()
        twist.linear.x = forward
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = rotation
        self.cmd_pub.publish(twist)
   
def main(args=None):
    rclpy.init(args=args)
    try:
        global node
        node = FollowArucoMarker()
    except Exception:
        rclpy.logging._root_logger.error(traceback.format_exc())
        rclpy.logging._root_logger.error('Failed to init Following aruco marker')
        rclpy.shutdown()
        sys.exit(1)

    try:
        rclpy.spin(node)
    except Exception:
        rclpy.logging._root_logger.error(traceback.format_exc())
    finally:
        node.emergency_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

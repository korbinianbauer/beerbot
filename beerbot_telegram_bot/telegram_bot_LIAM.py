import telebot
import os
import time
import sys
import arduino_utils
from programs import programs, program_descs

keep_alive = True

while keep_alive:
    try:
        bot = telebot.TeleBot("752116416:AAGMJL46RAjsSZ0keL9-_-PeYXK35VEpSI8")

        @bot.message_handler(commands=['start', 'help'])
        def send_welcome(message):
                bot.reply_to(message, "Howdy, how are you doing?2")
                print(message.chat.id)

        @bot.message_handler(commands=["list"])
        def echo_all(message):
                prg_list = "\n".join(program_descs)
                bot.reply_to(message, prg_list)

        @bot.message_handler(commands=["rec"])
        def echo_all(message):
                program = message.text.replace("/rec ", "")
                bot.reply_to(message, "Running program " + str(program))
                os.system("python /home/pi/beerbot/Rec_program.py " + str(program))
                bot.reply_to(message, "Done.")
            
                
        @bot.message_handler(commands=["sensors"])
        def echo_all(message):
                d = arduino_utils.get_sensors()
                answer = "\n".join([str(item) for item in d.items()])
                bot.reply_to(message, answer)
                
        @bot.message_handler(commands=["stop"])
        def echo_all(message):
                bot.reply_to(message, "Stopping Telegram Bot")
                global keep_alive
                keep_alive = False
                sys.cause_an_error()
                
                
        @bot.message_handler(commands=["shutdown"])
        def echo_all(message):
                bot.reply_to(message, "Raspberry Pi shutting down in 10s")
                time.sleep(10)
                os.system("echo qweasdyxc | sudo -S shutdown -h now")
                
        @bot.message_handler(commands=["reboot"])
        def echo_all(message):
                bot.reply_to(message, "Raspberry Pi rebooting")
                time.sleep(5)
                os.system("echo qweasdyxc | sudo -S reboot")

        @bot.message_handler(func=lambda message: True)
        def echo_all(message):
                bot.reply_to(message, message.text)
                


        bot.send_message(12272874, "BeerBot TelegramBot now running!")

        bot.polling()
        

    except Exception:
        if keep_alive:
            print("Failed, retrying in 10s")
            time.sleep(10)
        else:
            sys.exit()

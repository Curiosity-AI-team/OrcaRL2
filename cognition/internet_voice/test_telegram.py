import telebot

bot = telebot.TeleBot("7238562180:AAGt88x25LzLTBnoxzVEj8kGmGWXybzxyZg")
bot.remove_webhook()

@bot.message_handler(commands=['start', 'help'])
def send_welcome(message):
	bot.reply_to(message, "Howdy, how are you doing?")

@bot.message_handler(func=lambda message: True)
def echo_all(message):
	bot.reply_to(message, message.text)

bot.infinity_polling()
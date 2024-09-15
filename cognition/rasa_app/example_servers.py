import asyncio
import logging
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from vosk import Model, KaldiRecognizer
import json
import threading
import telebot

app = FastAPI()

# Load Vosk model (ensure you have downloaded the model and placed it in the "model" directory)
try:
    model = Model("D://Coding_AI//Internet_voice//mux_server//model//")
    logging.info("Vosk model loaded successfully.")
except Exception as e:
    logging.error(f"Could not load Vosk model: {e}")

# Telegram Bot setup
bot_token = "7238562180:AAGt88x25LzLTBnoxzVEj8kGmGWXybzxyZg"  # Replace with your Telegram bot token
bot = telebot.TeleBot(bot_token)
bot.remove_webhook()

# Global queues to store messages
telegram_message_queue = asyncio.Queue()
websocket_message_queue = asyncio.Queue()
audio_transcription_queue = asyncio.Queue()

# Start the Telebot in a separate thread
def start_telebot():
    @bot.message_handler(commands=['start', 'help'])
    def send_welcome(message):
        bot.reply_to(message, "Howdy, how are you doing?")

    @bot.message_handler(func=lambda message: True)
    def handle_message(message):
        # Put the message text into the queue
        asyncio.run_coroutine_threadsafe(
            telegram_message_queue.put(message.text),
            asyncio.get_event_loop()
        )
        bot.reply_to(message, f"Received your message: {message.text}")

    bot.infinity_polling()

telebot_thread = threading.Thread(target=start_telebot, daemon=True)
telebot_thread.start()

# Graceful shutdown
@app.on_event("shutdown")
def shutdown_event():
    bot.stop_polling()
    logging.info("Stopping Telebot polling...")
    if telebot_thread.is_alive():
        telebot_thread.join()
    logging.info("Telebot thread stopped.")

@app.websocket("/TranscribeStreaming")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    recognizer = KaldiRecognizer(model, 16000)
    audio_queue = asyncio.Queue()

    try:
        # Task to receive messages from the client (both audio and text)
        async def receive_messages():
            try:
                while True:
                    message = await websocket.receive()
                    if "bytes" in message:
                        audio_chunk = message["bytes"]
                        await audio_queue.put(audio_chunk)
                    elif "text" in message:
                        text_message = message["text"]
                        logging.info(f"Received text message from client: {text_message}")
                        if text_message == "submit_response":
                            # Signal to stop the audio stream
                            await audio_queue.put(None)  # Use None to signal end of stream
                            break
                        else:
                            # Put text messages into the websocket message queue
                            await websocket_message_queue.put(text_message)
                            await websocket.send_text(f"Echo: {text_message}")
            except WebSocketDisconnect:
                logging.info("WebSocket disconnected in receive_messages")
            except Exception as e:
                logging.error(f"Error in receive_messages: {e}")

        # Task to process audio data and perform speech recognition
        async def process_audio():
            try:
                while True:
                    audio_chunk = await audio_queue.get()
                    if audio_chunk is None:
                        break  # End of stream
                    if recognizer.AcceptWaveform(audio_chunk):
                        result = recognizer.Result()
                        result_json = json.loads(result)
                        text = result_json.get('text', '')
                        if text:
                            await audio_transcription_queue.put(text)
                            await websocket.send_text(f"Transcription: {text}")
                    else:
                        partial_result = recognizer.PartialResult()
                        result_json = json.loads(partial_result)
                        text = result_json.get('partial', '')
                        if text:
                            await websocket.send_text(f"Partial: {text}")
                # Send final result
                final_result = recognizer.FinalResult()
                result_json = json.loads(final_result)
                text = result_json.get('text', '')
                if text:
                    await audio_transcription_queue.put(text)
                    await websocket.send_text(f"Final Transcription: {text}")
            except Exception as e:
                logging.error(f"Error in process_audio: {e}")

        # New Task: Handle all messages in one function
        async def handle_all_messages():
            try:
                while True:
                    # Wait for any message from the queues
                    tasks = [
                        asyncio.create_task(telegram_message_queue.get()),
                        asyncio.create_task(websocket_message_queue.get()),
                        asyncio.create_task(audio_transcription_queue.get())
                    ]
                    done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
                    for task in done:
                        message = task.result()
                        # Process the message as needed
                        logging.info(f"Processing message: {message}")
                        # For example, you can send it back to the client
                        await websocket.send_text(f"Processed Message: {message}")
                    for task in pending:
                        task.cancel()
            except Exception as e:
                logging.error(f"Error in handle_all_messages: {e}")

        receive_task = asyncio.create_task(receive_messages())
        process_task = asyncio.create_task(process_audio())
        handle_messages_task = asyncio.create_task(handle_all_messages())
        await asyncio.gather(receive_task, process_task, handle_messages_task)
    except WebSocketDisconnect:
        logging.info("WebSocket disconnected")
    except Exception as e:
        logging.error(f"Unexpected error: {e}")
    finally:
        await websocket.close()

if __name__ == "__main__":
    import uvicorn

    logging.basicConfig(level=logging.INFO)

    def run():
        uvicorn.run(app, host="0.0.0.0", port=8000, ws_ping_interval=None)

    try:
        run()
    except KeyboardInterrupt:
        logging.info("Shutting down server...")
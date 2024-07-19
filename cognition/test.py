import json
import re
import urllib.request

# URL базы данных объектов
database_url = "https://raw.githubusercontent.com/Curiosity-AI-team/OrcaRL2/main/cognition/database/output.json"

# Загрузка базы данных объектов
with urllib.request.urlopen(database_url) as url:
    objects_db = json.loads(url.read().decode())

# Функции действий робота
move_actions = {
    "move_head": "self.move_head",
    "arm_random": "self.arm_random",
    "arm_touch": "self.arm_touch",
    "arm_grasp": "self.arm_grasp",
    "arm_put": "self.arm_put",
    "arm_point": "self.arm_point",
    "arm_move": "self.arm_move",
    "arm_rest": "self.arm_rest",
    "body_sit": "self.body_sit",
    "body_run": "self.body_run",
    "body_walk": "self.body_walk",
    "body_crawl": "self.body_crawl",
    "body_stand": "self.body_stand",
    "body_lie_down": "self.body_lie_down",
    "body_fall_over": "self.body_fall_over",
    "leg_jump": "self.leg_jump",
    "leg_kick": "self.leg_kick",
}

# Функция для проверки наличия маркера в базе данных
def marker_exists(marker_id, objects_db):
    for obj in objects_db:
        for key, value in obj.items():
            if str(value.get("target")) == marker_id:
                return True
    return False

# Функция генерации команд
def generate_robot_commands(user_input):
    commands = []
    
    # Стандартное действие перед выполнением команд
    commands.append("body_stand")
    
    # Поиск маркера
    marker_match = re.search(r'маркер\s*(\d+)', user_input, re.IGNORECASE)
    if marker_match:
        marker_id = marker_match.group(1)
        if marker_exists(marker_id, objects_db):
            if "подойти" in user_input.lower():
                commands.append(f"body_walk to the target={marker_id}")
            elif "принеси" in user_input.lower():
                commands.extend([
                    f"body_walk to the target={marker_id}",
                    f"arm_grasp to the target={marker_id}",
                    "body_walk to the target=human",
                    "arm_put to the target=table near target=human"
                ])
            return commands
    
    # Другие команды
    if "приготовь мне бутер" in user_input:
        return ["Sorry, I can't do that"]
    
    if "станцуй" in user_input:
        commands.append("arm_random")
        return commands
    
    # Если команда не распознана
    return ["Sorry, I can't do that"]

# Примеры использования
user_input1 = "подойти к маркеру 82"
user_input2 = "Принеси мне маркер 82"
user_input3 = "приготовь мне бутер"
user_input4 = "Станцуй"

print(generate_robot_commands(user_input1))
print(generate_robot_commands(user_input2))
print(generate_robot_commands(user_input3))
print(generate_robot_commands(user_input4))



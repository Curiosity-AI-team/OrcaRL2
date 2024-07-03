import spacy
import nltk
import yaml

nltk.download('punkt')
nltk.download('averaged_perceptron_tagger')

# Load spaCy model
# python3 -m spacy download en_core_web_sm
nlp = spacy.load("en_core_web_sm")

# Load body parts and actions from YAML file
with open('body_parts_actions.yaml', 'r') as file:
    body_parts_actions = yaml.safe_load(file)

def process_input(input_text):
    doc = nlp(input_text.lower())
    body_part_found = None
    action_found = None

    for token in doc:
        if token.lemma_ in body_parts_actions:
            body_part_found = token.lemma_
        for body_part, actions in body_parts_actions.items():
            if token.lemma_ in actions:
                action_found = token.lemma_
                if body_part_found and action_found in body_parts_actions[body_part_found]:
                    return f"The action '{action_found}' is valid for the body part '{body_part_found}'."
    
    return "No valid body part or action found in the input."

# Example usage
print(process_input("Can you blink your eyes?"))
print(process_input("I will wave my hand."))
print(process_input("She likes to kick with her arms."))

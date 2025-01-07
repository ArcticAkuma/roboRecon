import yaml

with open('config.yml', 'r') as file:
    config = yaml.safe_load(file)

    print(config['donkey-server']['hostname'])
    print(config['donkey-server']['port'])
    print(config['yolo']['console-logging'])
import configparser
import json

config = configparser.ConfigParser()
config.read('firmware/platformio.ini')

environments = []
for section in config.sections():
    if section.startswith("env:"):
        env_name = section.split("env:")[1]
        environments.append({"env": env_name})

print(json.dumps(environments))

import configparser
import json

""" This script parses the platformio.ini file and outputs a JSON array of environments to be used by GitHub Actions."""

# Parse the platformio.ini file
config = configparser.ConfigParser()
config.read('firmware/platformio.ini')

# Extract the environment names from the sections
environments = []
for section in config.sections():
    if section.startswith("env:"):
        env_name = section.split("env:")[1]
        if "teensy" in env_name or env_name == "dev" or env_name == "vattenkar":
            continue  # Skip environments that contain "teensy"
        environments.append({"env": env_name})

# Output the environments as a JSON array
print(json.dumps(environments))

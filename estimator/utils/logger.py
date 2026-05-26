import sys

class Logger:
    RESET = "\033[0m"
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"

    @staticmethod
    def info(message):
        print(f"{Logger.GREEN}[INFO]: {message}{Logger.RESET}", file=sys.stdout)

    @staticmethod
    def debug(message):
        print(f"{Logger.BLUE}[DEBUG]: {message}{Logger.RESET}", file=sys.stdout)

    @staticmethod
    def error(message):
        print(f"{Logger.RED}[ERROR]: {message}{Logger.RESET}", file=sys.stderr)


from datetime import datetime
from termcolor import colored
import sys

class HelperLog:
    @staticmethod
    def flush():
        """Flush the output buffer."""
        sys.stdout.flush()

    @staticmethod
    def notify_action(action_type, msg, show_when=True):
        """Notify user of an action being taken."""
        if not show_when:
            return
        print(colored(f'👋🏻({action_type})', 'yellow'), end='')
        print(colored(f' {msg}', 'white'))
        HelperLog.flush()

    @staticmethod
    def notify_success(action_type, msg, show_when=True):
        """Notify user of a successful action."""
        if not show_when:
            return
        print(colored(f'🥳({action_type})', 'green'), end='')
        print(colored(f' {msg}', 'white'))
        HelperLog.flush()

    @staticmethod
    def notify_warning(action_type, msg, show_when=True):
        """Notify user of a warning."""
        if not show_when:
            return
        print(colored(f'🚨({action_type})', 'yellow'), end='')
        print(colored(f' {msg}', 'white'))
        HelperLog.flush()

    @staticmethod
    def notify_error(action_type, msg, show_when=True):
        """Notify user of an error."""
        if not show_when:
            return
        print(colored(f'💢({action_type})', 'red'), end='')
        print(colored(f' {msg}', 'white'))
        HelperLog.flush()

    @staticmethod
    def notify_interrupt(filedir, show_when=True):
        """Notify user of an interruption."""
        if not show_when:
            return
        print(colored(f'\n💀(interrupted)', 'magenta'), end='')
        print(colored(f' {filedir}', 'cyan'))
        HelperLog.flush()

    @staticmethod
    def notify_start(filedir, show_when=True):
        """Notify user that a process has started."""
        if not show_when:
            return
        print(colored(f'🚦(started)', 'magenta'), end='')
        print(colored(f' {filedir}', 'cyan'))
        HelperLog.flush()

    @staticmethod
    def notify_termination(filedir, show_when=True):
        """Notify user that a process has terminated."""
        if not show_when:
            return
        print(colored(f'🚦(terminated)', 'magenta'), end='')
        print(colored(f' {filedir}', 'cyan'))
        HelperLog.flush()

    @staticmethod
    def notify_update(action_type, msg, show_when=True):
        """Notify user of an update with a timestamp."""
        if not show_when:
            return
        cur_time = datetime.now()
        timestamp = f"{cur_time.hour:02}:{cur_time.minute:02}:{cur_time.second:02}:{cur_time.microsecond // 1000:03}"
        
        print(colored(f'⏰️ [{timestamp}]', 'green'), end='')
        print(colored(f' ({action_type})', 'yellow'), end='')
        print(colored(f' {msg}', 'white'), end='')
        print()
        HelperLog.flush()


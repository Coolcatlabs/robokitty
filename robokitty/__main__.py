"""
robokitty – entry point and CLI dispatcher.

Parses command-line arguments and dispatches to the appropriate subsystem.
All exit codes are defined by ExitCode and returned to the shell via
SystemExit at the bottom of the module.

Commands
--------
    robokitty gait [crawl|walk|run]       Run a gait pattern (default: crawl)
    robokitty servo diagnose <id>         Print full register diagnostic
    robokitty servo repair <id>           Attempt software recovery

Global options apply to all commands:
    --port   Serial port (default: DEFAULT_PORT)
    --baud   Baud rate   (default: DEFAULT_BAUD)
"""

from enum import IntEnum

from ._robokitty import (
      QuadrupedWalker, 
      get_servo_metrics, 
      repair_servo, 
      set_servo_id, 
      AX12Interface,
      )
from ._cli import _cli_parser


class ExitCode(IntEnum):
    SUCCESS = 0
    FAILURE = 1


def main() -> ExitCode:
    args = _cli_parser()

    if args.command == "gait" or args.command is None:
        pattern = getattr(args, "pattern", "crawl")
        with QuadrupedWalker(
            port=args.port,
            baudrate=args.baud,
            gait=pattern,
        ) as walker:
            walker.loop()
        return ExitCode.SUCCESS

    elif args.command == "servo":
        if args.servo_command is None:
            print("Usage: robokitty servo <diagnose|repair|set-id> <id>")
            return ExitCode.FAILURE

        if args.servo_command == "set-id":
            with AX12Interface(args.port, args.baud) as ax:
                return ExitCode.SUCCESS if set_servo_id(ax) else ExitCode.FAILURE

        if not 0 <= args.id <= 253:
            print(f"Error: servo ID must be between 1 and 253, got {args.id}")
            return ExitCode.FAILURE

        with AX12Interface(args.port, args.baud) as ax:
            if args.servo_command == "diagnose":
                return (
                    ExitCode.SUCCESS
                    if get_servo_metrics(ax, args.id)
                    else ExitCode.FAILURE
                )
            elif args.servo_command == "repair":
                return (
                    ExitCode.SUCCESS if repair_servo(ax, args.id) else ExitCode.FAILURE
                )
            elif args.servo_command == "set-id":
                return ExitCode.SUCCESS if set_servo_id(ax) else ExitCode.FAILURE

    print(f"Unknown command: {args.command}")
    return ExitCode.FAILURE


if __name__ == "__main__":
    raise SystemExit(main())

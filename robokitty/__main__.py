from ._robokitty import QuadrupedWalker
from ._cli import _cli_parser


def main():
    args = _cli_parser()

    with QuadrupedWalker(
        port=args.port,
        baudrate=args.baud,
        gait=args.gait,
    ) as walker:
        walker.loop()


if __name__ == "__main__":
    raise SystemExit(main())

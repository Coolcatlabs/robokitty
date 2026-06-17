from ._robokitty import QuadrupedWalker


def main():
    # args = _cli_parser()

    with QuadrupedWalker() as walker:
        walker.loop()


if __name__ == "__main__":
    raise SystemExit(main())

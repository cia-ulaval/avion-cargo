import click
from loguru import logger

@click.command(context_settings={"help_option_names": ["-h", "--help"]})
@click.option()
@logger.catch
def main():
    pass
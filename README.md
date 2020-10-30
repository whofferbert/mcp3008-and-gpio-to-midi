# mcp3008-and-gpio-to-midi
A program to read analog inputs from an MCP3008, and some GPIO pins, and translate that all to midi stuff.

## TODO
a description about the submodule
learn how to do submodules right

## crontab use
This is meant to be permanently attached to a raspberry pi that does stuff with midi.
As such, it's proper to start it at boot time, here's one way:

```bash
sudo /home/modep/guitar/mcp3008_and_gpio_to_midi/mcp3008-and-gpio-to-midi.py 2>>/home/modep/mcp_gpio_err.log 1>/dev/null &
```

Note that you will want to have nopasswd sudo set up for the user running this.

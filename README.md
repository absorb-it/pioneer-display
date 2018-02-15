# pioneer remote display for DEH-X7800DAB

This is a library to add a remote display to a DEH-X7800DAB radio. The data is internally transferred via SPI, so you can easily use this data and attach a display at another place. You just have to know how to interpret the datastream - and that's what this library is all about.

[Checkout the wiki](https://github.com/absorb-it/pioneer-display/wiki) for more information and how I managed to interpret the datastream - maybe this helps you to adapt the code for any other Car Hifi Systems.

And at the end you might add some buttons, use the [remote control hack as described there](http://www.jvde.net/node/7) and have your second controller for your radio.


***
The library will mirror the display one to one. Below is an image of the pioneer car hifi with nearly all display items light up (it was used to figure out what is possible at all) and the selfmade external controller beside during a different time (with some remote control buttons and the mirrored display part below).

![Pioneer Radio][pioneer]
![Pioneer Remote Display][pioneer_remote]

***


[pioneer]: https://raw.githubusercontent.com/wiki/absorb-it/pioneer-display/images/pioneer.jpg
[pioneer_remote]: https://raw.githubusercontent.com/wiki/absorb-it/pioneer-display/images/pioneer_remote.jpg

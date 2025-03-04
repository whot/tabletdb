# tabletdb

tabletdb is a Rust crate and associated utilities to provide the functionality
of [libwacom](https://github.com/linuxwacom/libwacom).

It provides **static** information about graphics tablets (Wacom, XP-Pen, Huion, ...) to
help with UI decisions. libwacom for example is used by GNOME to determine if a tablet
needs to be mapped to a monitor (and which monitor) or which buttons on a stylus are
actually configurable.

tabletdb doesn't have any direct effect on functionality, it is merely a wrapper
around the text files that specify the various tablet features.

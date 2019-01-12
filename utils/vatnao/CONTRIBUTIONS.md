# Contribution Guide:

In the interest of keeping the vatnao codebase clean, changes to vatnao must be done as following.

## Data:

Any data to be passed through to the front end must be added to the `VatnaoFrameInfo` struct. Processing of that data should be performed in `generateFrameInfo`, a function that has access to the `blackboard` and the `Vision` class. Where possible, use data from `blackboard` instead of `Vision` so we don't have to worry about changes to the `Vision` Schema.

Data passed through `VatnaoFrameInfo` must be complete and usable as is. There shouldn't need to be additional processing on any data in the front end. Data must also be completely abstracted away from rUNSWift types. e.g. we can't return a `RegionI`, `RANSACLine`, or `uint8_t*` type. Instead, those should be abstracted away as `FrameRect`, `FrameLine` or `RgbImg` so there is a consistent interface in the front end and so that changes to rUNSWift types have a minimised and maintainable impact on vatnao.

All data code is in `utils/vatnao/app`.

## Front end:

The only access to the program that a front end has is via the `AppAdaptor`. The `AppAdaptor` allows you to move forward and back frames, and to generate the current frame. The front end must not try to access the `blackboard` or the `Vision` class. The front end is only for interpreting the data in `VatnaoFrameInfo`, which should be in a useful, processed form when received.

Any unique method for accessing the data should be kept in it's own unique sub folder. For example, we currently have `utils/vatnao/ui` for viewing data on the desktop, but if we were to add, say a web front end, that should go in a folder such as `utils/vatnao/web`. Access to any different configuration should be managed by command line flags set in `options.cpp` and `main.cpp`.

### UI

UI is a QT implementation of the vatnao front end. Any interaction with `AppAdaptor` is defined as slots in `VatnaoManager` and the ui is defined in `ui.cpp`. The UI should be completely scalable and responsive. To make responsive images, use the `ImageView` helper class in `uiElements/imageView.hpp`, which will automatically scale, while keeping images in the correct aspect ratio.

----

Thank you for taking the time to improve vatnao. I hope it'll be useful as you work on rUNSWift.

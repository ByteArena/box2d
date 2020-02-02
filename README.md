# Box2D.go ![](https://travis-ci.org/ByteArena/box2d.svg?branch=master)

## What is this ?

This is Go a port of Box2D (https://github.com/erincatto/Box2D), a 2D physics engine for games written in C++ by Erin Catto.

The port is complete and based on the latest Box2D commit as of 2017-09-20 (https://github.com/erincatto/Box2D/commit/f655c603ba9d83f07fc566d38d2654ba35739102)

## Who did this ?

The ByteArena team did. Erin Catto, the author of the original software, was not involved in the port.

https://s3.eu-central-1.amazonaws.com/bytearena-public/ba-prod-twitter.mp4

## Documentation

We kept the source code as close to the C++ as we possibly could. So the documentation you'll find on Box2D is relevant.

https://box2d.org/documentation/

## API changes

The API had to change a tiny bit due to the fact that :

* Go has no constructors as a language feature, thus code for constructors has been placed in `Make$NAME_OF_TYPE` functions
* Go has no support for function overloading; some functions implemented multiple times for different sets of parameters under the same name in C++ are distinguished by name in the golang version; we tried to make names explicit so that should not be an issue
* Go has no support for operator overloading; this C++ feature is used extensively throughout the C++ version of Box2D (mainly for the vector and matrix arithmethic), and has been converted to good old, albeit verbose function calls

## Tests

No opengl testbed for the moment.

Our tests verify the output of position and rotation of bodies over time against those generared by the C++ reference.

Right now, there's a test (passing) checking all the supported body shape collisions in `cmd/test-character-collision`.

## Usage example

Have a look at `cpp_compliance_test.go`.

## License of the original Box2D (C++)

The original Box2D is developed by Erin Catto, and has the zlib license. Thank you Erin for this incredible piece of software.

## License of this port (Go)

Box2D.go is developed by ByteArena (https://github.com/bytearena), and has the zlib license. While the zlib license does not require acknowledgement, we encourage you to give credit to Box2D.go in your product.

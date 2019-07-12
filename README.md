# Bagmetti

<div style="margin: 0 auto">
  <img src="images/bagmetti.jpg" alt="Bagmetti processing a bag fie"/>
</div>

**BAG** file utility for **ME**ssage, **T**f, and **TI**me.

[![Build Status](https://travis-ci.com/naoki-mizuno/bagmetti.svg?branch=master)](https://travis-ci.com/naoki-mizuno/bagmetti)

## What's This?

Bagmetti is a collection of utility scripts for filtering and modifying
existing [bag files](http://wiki.ros.org/Bags). You can use Bagmetti, for
example, when:

- `rosbag record -a` takes up too much storage and you want a more lightweight
  bag file with fewer topics
- you want to exclude certain TF transformations
- creating a bag file for only a certain time range
- you want to rename existing TF frame names

## But what about...

Yes, there are existing alternatives, but sometimes they aren't good enough.


### `rosbag filter`

Problem: `rosbag filter` only accepts one argument for filtering messages.

The argument is a Python one-liner. When you have multiple topics and TF
transformations to consider, this becomes a huge string full of `and`s and
`or`s.

Using `rosbag filter` also makes it harder to track down what was used for
filtering a certain bag file. Depending on your shell configuration the
command you executed can be lost, and even if you have the command history you may
not be able to tell what expression was used last. What's more, one-line
expressions are hard to read.

Bagmetti lets you manage filtering rules in a YAML file. Therefore, you could
put it in the same directory as the bag file so that you can later look back
at what was used to filter the original bag file.


### `rosbag play -s 42 -u 60 foo.bag`

Problem: The format is `start time + delta`

You can specify what time to start and **how long** to play instead of what
time to end. Most of the times you know the start and end time of the section
you want to cut out. In these cases it's more intuitive to specify the start
and end time, and Bagmetti let's you do that.


### `rosbag play foo.bag some_topic:=new_topic_name`

Problem: Can't rename TF transformations

You can rename topic names using the remapping command line arguments or
`<remap from="some_topic" to="new_topic_name" />` in launch files. However,
you can't rename TF transformations with this method. Also, often times you
forget to do this, and it could be a pain in the neck to specify these
remapping arguments every time.


## Running

Filter in or out certain messages from a bag file and output to a different bag file.

```
rosrun bagmetti filter.py <bag file path> <output file path> <config file path>
```

Rename topic names and TF transformations.

```
rosrun bagmetti rename.py <bag file path> <output file path> <config file path>
```


## Config File Format

Formats are different depending on what script you use (don't worry; they're
really simple and written in YAML).

- [Filter](./docs/filter.md)
- [Rename](./docs/rename.md)

Also check out the [sample config files](./samples).


## TODO

- Unit tests
- Python expression feature for `filter.py`

```yaml
topic:
  -:
    # Exclude if message matches the expression
    - name: /imu/data
      expr: msg.linear_acceleration.x < 1.5
    - velodyne_points

tf:
  +:
    # Include if transformation matches the expression
    - name: map -> odom
      expr: tf.translation.x > 0.5
```


## License

MIT

## Author

Naoki Mizuno (mizuno.naoki@rm.is.tohoku.ac.jp)

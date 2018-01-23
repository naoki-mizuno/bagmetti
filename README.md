# Bagmetti

<div style="margin: 0 auto">
  <img src="images/bagmetti.jpg" alt="Bagmetti processing a bag fie"/>
</div>

**BAG** file utility for **ME**ssage, **T**f, and **TI**me.


## What's This?

Bagmetti is a utility script for filtering and modifying an existing [bag
file](http://wiki.ros.org/Bags) and create a new bag file. You can use
Bagmetti, for example, when:

- you've done `rosbag record -a` to collected all data in an experiment, but
  only use a few topics for post-processing
- you want a lightweight bag file without the Velodyne point cloud topic
- running amcl on a bag file that was created while running gmapping (which
  means you need to exclude the `map` to `odom` transformations done by
  gmapping)
- creating a bag file for only a certain time range (more solid method than
  `rosbag play -s 42 -u 60`)


### But what about `rosbag filter`?

The problem with `rosbag filter` is that it only accepts one argument, which
is a Python expression, for filtering messages out. This quickly becomes a
one huge string full of `and`s and `or`s, and is pretty much unmanageable.

Using `rosbag filter` also makes it harder to track down what was used for
filtering a certain bag file. Depending on your shell configuration the
command history can be lost, and even if you have the command history you may
not be able to tell what expression was used last. What's more, one-line
expressions are hard to read.

Meet Bagmetti, which lets you manage filtering rules in a YAML file.
Therefore, you could put it in the same directory as the bag file and even
version control it if you with to.


## Running

Filter in or out certain messages from a bag file and output to a different bag file.

```
rosrun bagmetti filter.py <bag file path> <output file path> <config file path>
```


## Config File Format

Config files use YAML. Here are some samples to get you started:

```yaml
# Create a bag file that can be used for gmapping from a bag file
# with all sensor data + running gmapping while collecting data
tf:
  exclude:
    # gmapping does this transformation
    - map -> odom
    # GPS-based gmapping
    - gps_origin -> gps_antenna
    - gps_antenna -> gps_base_link
    - map -> odom
topic:
  include:
    - /velodyne_points
    - /odom
    - /vg440/imu/data
    - /cnt723/count
    - /ssv102/nmea_sentence
    - /javad/nmea_sentence
    - /cnt723/count
time:
  default: exclude
  include:
    - 30 -> 60
```

`include` can be written as `+`, and `exclude` can be `-`. It can even be just
the first character (i.e. `i` or `e`)! By default, if only either of `include` or
`exclude` is specified in the message type (`tf`, `topic`, `time`), like in
the example, the unspecified messages implicitly becomes the other. You can,
however, explicitly specify the default:

```yaml
# amcl
tf:
  # Explicitly include unspecified TF transformations
  default: +
  # Tip: Doesn't have to be a list
  -: /map, /odom
topic:
  # Implicitly exclude unspecified topics
  +: ['odom', 'scan']
```

Be careful when specifying `-` in `default`:

```yaml
# This is invalid YAML!!
default: -
# Make sure to quote!
default: '-'
```

I recommend that you don't specify the `default` and just use implicit
inclusion/exclusion.


### Rule Types

Following sections describe how a rule can be specified.

#### Topics

The topic name, with or without slash.


#### TF

Rule type: `tf`

The rule for TF is in the following format:

```yaml
tf:
  include: map -> odom
```

This will include TF transformations that has `map` as the parent frame ID and
`odom` as the child frame ID. `,` or ` ` (space) can be used instead of a
`->`. Either of the frame names can be omitted:

```yaml
tf:
  -: base_link ->
```

This will exclude any TF message that has `base_link` as the parent name.


#### Time

Rule type: `time`

Specifies the range of the time, starting from the beginning of the bag file.

Follows the same format as TF. For example,

```yaml
time:
  include:
    32.2 -> 45
```

will only include messages and transformations published within this time range.

As is the case for TF, either the start time or end time can be omitted.


## TODO

- `alter.py`, which lets you modify the messages in a bag file
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

# Filter Config Format

Config files use YAML. Here are some samples to get you started:

## GMapping

```yaml
tf:
  exclude:
    - map -> odom
topic:
  include:
    - /scan
    # If you want te recreate /scan from 3D point clouds
    # - /velodyne_points
    - /odom
    - /imu
```

## AMCL

```yaml
tf:
  exclude:
    - map -> odom
topic:
  include:
    - /map
    - /odom
    - /scan
```

## More on What You Can Do

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

Be careful when you have topics that are only published once at the beginning
of the bag file (for example, latched topics such as `/map`). When you trim
the bag file not starting from the beginning, these topics are not included in
the new bag file.

# Bagmetti

<div style="margin: 0 auto">
  <img src="images/bagmetti.jpg" alt="Bagmetti processing a bag fie"/>
</div>

BAG file utility for MEssage, Tf, and TIme.


## Running

Filter in or out certain messages from a bag file and output to a different bag file.

```
rosrun bagmetti filter.py <bag file path> <output file path> <config file path>
```


## Config File Format

Config files use YAML. Here are some samples to get you started:

```yaml
# gmapping
tf:
  default: include
  exclude:
    # gmapping does this transformation
    - map -> odom
    # GPS-based gmapping
    - gps_origin -> gps_antenna
    - gps_antenna -> gps_base_link
    - map -> odom
topic:
  default: exclude
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
the first character (i.e. `i` or `e`). If only either of `include` or
`exclude` is specified, the `default` implicitly becomes the other. You can,
however, explicitly specify the default.

```yaml
# amcl
tf:
  default: +
  -:
    - /map, /odom
topic:
  # Implicit default: exclude
  +:
    - odom
    - scan
```

Be careful when specifying `-` in `default`:

```
# This is invalid YAML!!
default: -
# Make sure to quote!
default: '-'
```

The conclusion is: don't go through the trouble of explicitly specifying the
`default`.

### Rule Types

#### Topics

The topic name, with or without slash.


#### TF

Rule type: `tf`

The rule for TF is in the following format:

```
[parent frame][separator][child frame]
```
Separator: `,`, ` ` (space), `->`

For example,

```yaml
tf:
  include: map -> odom
```

will include TF transformations that has `map` as the parent frame ID and
`odom` as the child frame ID. Either of the frame names can be omitted (e.g.
`-tf: map ->` to exclude any TF message that has `map` as the parent name).

```yaml
tf:
  -: base_link ->
```


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


## License

MIT

## Author

Naoki Mizuno (mizuno.naoki@rm.is.tohoku.ac.jp)

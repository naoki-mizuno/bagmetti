# Bagmetti

BAG file utility for MEssage, Tf, and TIme.


## Running

```
rosrun bagmetti filter.py <bag file path> <output file path> <config file path>
```

or simply,

```
filter.py <bag file path> <output file path> <config file path>
```


## Config File Format

A line in the config file should be in the following format:

```
[enforcement][rule_type]: [rule]
```


### Enforcement

Enforement specifies whether to include or exclude the pattern that follows.

- Include: `+`
- Exclude: `-` or `!`

By default, it will include but you can change the default enforcement by
using the `default` specifier. You can pass `plus`, `minus`, `include`,
`exclude` to the `default` specifier.

```
default: -
# Do a bunch of exclusion
default: +
# Then switch back to inclusion to do some inclusions
```


### Rule Types

#### Topics

Rule type: `topic`

The topic name can (but doesn't have to) start with a `/`. If you specify a
topic for both inclusion and exclusion (i.e. `+` and `-` in the same
configuration file), inclusion will be prioritized.


#### TF

Rule type: `tf`

The rule for TF is is the following format:

```
[parent frame][separator][child frame]
```
Separator: `,`, ' ', `->`

For example,

```
+tf: map -> odom
```

will include TF transformations that has `map` as the parent frame ID and
`odom` as the child frame ID. Either of the frame names can be omitted (e.g.
`-tf: map ->` to exclude any TF message that has `map` as the parent name).


#### Time

Rule type: `time`

Specifies the range of the time, starting from the beginning of the bag file.

Follows the same format as TF. For example,

```
+time: 32.2 -> 45
```

will only include messages and transformations published within this time range.

As is the case for TF, either the start time or end time can be omitted.


## License

MIT

## Author

Naoki Mizuno (mizuno.naoki@rm.is.tohoku.ac.jp)

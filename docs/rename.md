# Rename Config Format

Config files use YAML. Aside from specifying the rename rules separately for
`topic` and `tf`, there are only two tags in the config files: `from` and
`to`.

If there are multiple `from` matches, only the first one is applied.

## Basic Usage

The following will replace all occurrences of `foo` with `bar`.

```yaml
topic:
  from: foo
  to: bar
```

For example, `/my_foo/bar/baz/foo` becomes `/my_bar/bar/baz/bar`.

## Renaming TF frame IDs

You can rename the `frame_id` fields in normal messages and TF messages.

```yaml
tf:
  - from: foo/map
    to: map
  - from: foo/odom
    to: odom
```

For a bag file of a typical SLAM process, this will rename two things: the
`header.frame_id` in the `/map` topic's OccupancyGrid, and the TF
transformations between `foo/map` and `foo/odom`.

## Wild Cards

You can use asterisks (`*`) and pluses (`+`) for wild cards. The difference
between `*` and `+` is that `+` doesn't match empty strings.

```yaml
topic:
  from: /velodyne*/foo
  to: /my_velodyne
# vs
topic:
  from: /velodyne+/foo
  to: /my_velodyne
```

With `*`, `/velodyne/foo` becomes `/my_velodyne` but with `+`,
`/velodyne/foo` is not a match, which means it won't be replaced (name remains
unchanged). However, `/velodyne_points/foo` becomes `/my_velodyne` in both
cases.

## Reusing Wild Card Matches

You can have multiple wild cards in the `from`, with matching `*` in `to`:

```yaml
topic:
  from: /foo/*/*/*
  to: /*/*/*/bar
```

`/foo/a/b/c` becomes `/a/b/c/bar`.

The wild card matches can also be reused with `{0}`, `{1}`, ... `{N}` where
`{N}` represents the Nth matching `*` or `+` in the `from` pattern. If you're
familiar with regular expressions, they're `\1`, `\2`, ... `\N` except they
start from 0. Typical usage of this is for reordering:

```yaml
topic:
  from: /foo/*/*/*
  to: /bar/{2}/{0}/{1}
```

`/foo/a/b/c` becomes `bar/c/a/b`.

Not all `*` have to be specified in `to`:

```yaml
topic:
  from: foo/*/*/*
  to: bar/{2}
```

`/foo/a/b/c` becomes `/bar/c`.

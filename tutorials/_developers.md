This page contains instructions for Drake developers & contributors, related to
improving the tutorials or adding new ones.

# Local Editing

To rebuild from source and run tutorials using Bazel, use e.g.:
```
bazel run //tutorials:mathematical_program
```

# Adding a New Tutorial

- Add a link to the notebook in `tutorials/index.ipynb`.

- In the pull request overview, include a `nbviewer` link to the directory on
your fork and branch, e.g.,
`https://nbviewer.jupyter.org/github/{user}/drake/blob/{branch}/tutorials/`

# Deploying changes

The tutorials on the Drake website are refreshed to latest master as part of
<<<<<<< HEAD
<<<<<<< HEAD
our [stable release process](/release_playbook.html). In general we do not
=======
our [stable release process](/release_playbook.html]. In general we do not
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
=======
our [stable release process](/release_playbook.html). In general we do not
>>>>>>> 65b76e12737b188b94fc473aa3d3c4fb4fea5a0f
upgrade them between those monthly releases, but in case of emergency feel
free to manually edit them online at deepnote.com. Any manual changes will
be overwritten during the next stable release, so be sure that the fixes
get merged to Drake master as well.

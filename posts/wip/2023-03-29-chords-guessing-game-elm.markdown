---
title: Chord Guessing Game in Elm
author: Sascha Bubeck
tags: Elm, elm-random, GUI
---

[^mark]
[Elm](https://www.url.org)
[`elm/random`](https://url).


## Set-up

To follow the blog post step by step, you will need the following setup:


<details>
<summary>
Tools/Libraries
</summary>

  * [elm/random](https://docs.haskellstack.org/en/stable/README/) - build system (lts-16.6)
  * [html](https://hackage.haskell.org/package/linear) - representation of two-dimensional vectors
  * [elm-ui](https://hackage.haskell.org/package/gloss) - visualization of particles
</details>


<details>
<summary>
Build steps
</summary>
```bash
elm init

elm install random
elm install..
```
</details>

<details>
<summary>
Imports in `Main.elm`
</summary>
```elm
module Main exposing (main)

import Array as A
import Browser
import Dict
import Element exposing (..)
import Element.Input as Input
import Random exposing (..)
```
</details>

Alternatively, if you don't want to copy all the
code snippets in this blog post, have a look at
[this repository](https://github.com/mkDoku/molecularDynamics).
Following the blog post
step-by-step should result in a working implementation.
If you are having trouble implementing this, feel free to [contact me](/contact.html).

## Introduction

## Jazz Chords

### Major Chords

### Minor Chords

## Model View Update Model in Elm

## References

  [^mark]: This is how we use citations

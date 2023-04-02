# mkDokuBlog
This is the code for building my blog.

# Install

```bash
cabal build
```

# Work with the blog

```bash
site watch
```

# Alternative build/watch

```bash
cabal exec site build

stack exec site build

nix-build release.nix; ./result/bin/site watch
```

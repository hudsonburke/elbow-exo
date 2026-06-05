# elbow-exo


Using PlatformIO for low-level development of the elbow exoskeleton firmware.

To get started, install [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html). If you prefer VS Code, install the [PlatformIO IDE extension](https://platformio.org/install/ide?install=vscode).

## Neovim / clangd

`clangd` needs a PlatformIO compilation database to resolve framework headers such as `Arduino.h`.

1. Generate `compile_commands.json` for the environment you are editing:
   - `platformio run -t compiledb -e teensy41`
   - replace `teensy41` with `uno` or `nanoatmega168` if needed
2. Restart `clangd` in Neovim after regenerating the database.
3. Re-run the command whenever you change PlatformIO environments or build flags.

This repository includes a shared `.clangd` file that points `clangd` at the project-root `compile_commands.json`, and the PlatformIO config exports toolchain include paths into that database.

If `clangd` still misses standard library headers from the PlatformIO toolchain, start it with a query-driver that trusts the PlatformIO compilers. Example `nvim-lspconfig` setup:

```lua
require("lspconfig").clangd.setup({
  cmd = {
    "clangd",
    "--background-index",
    "--query-driver=" .. vim.env.HOME .. "/.platformio/packages/**/bin/*",
  },
})
```

VS Code users can still open the project directly and let PlatformIO IDE manage the editor integration.
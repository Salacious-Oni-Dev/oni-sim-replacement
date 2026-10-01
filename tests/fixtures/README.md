# Pinned save blobs

Real save blobs written by earlier builds of this SimDLL, from small synthetic test worlds.

| File | Version | World |
|---|---|---|
| `plain-v15.blob` | `kSaveVersion` = 15 | a world that never activated volume-fractions |
| `gas-v16.blob` | `kSaveVersionGasMixture` = 16 | a world holding real gas-mixture state |
| `promoted-room-v17.blob` | `kSaveVersionRoomPromotion` = 17 | a world with a promoted room |
| `gas-v18.blob` | `kSaveVersionExtensions` = 18 | the `gas` world above |
| `promoted-room-v18.blob` | `kSaveVersionExtensions` = 18 | the promoted-room world |

## Why these are files and not a test that regenerates them

**Nothing in the current tree can write these versions any more.** `World::ToBlob` writes the
newest format only. A test that regenerated its own input would be testing the new writer
against the new reader, and would pass no matter what happened to the old formats.

These blobs are therefore the evidence that a save written by an earlier build still loads.

`plain-v15.blob` is the most important one despite being the least interesting. The others
prove the reader still **reads** what earlier builds wrote; this one proves that a world using
none of the extensions still **writes** exactly Klei's own format. The game's own DLL has to
keep accepting those bytes, which is a stronger claim than our reader agreeing with our writer.

## What they are used for

`vftest`'s "legacy blobs still load" arms. Point them elsewhere with `--fixtures <dir>`.

`vftest --dump-blob-dir <dir>` names each file it writes by the blob's **own** version, read
out of its header, so running it on a current build cannot overwrite these older formats.

If a fixture ever fails to load, that is a real finding about the reader, not a flaky test.
Never replace a fixture to make a failure go away.

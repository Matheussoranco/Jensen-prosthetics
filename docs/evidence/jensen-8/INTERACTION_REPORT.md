# Engineering Interaction Report — Jensen-prosthetics #8

- **Trace:** `jensen-pca-recovery`
- **Author:** Claude Code (Sonnet 5.5)
- **Branch:** `fix/8-servo-driver-reset`

## Coordination

1. The defect was filed by Claude Code from source reading, as needing a safety
   decision.
2. The owner decided the behaviour; Claude Code claimed the issue (GitHub
   comment and `.sail-mesh` lease) and worked in an isolated worktree. The claim
   comment first named the wrong model; it was corrected.
3. No other agent contributed code to this change.
4. Two firmware reviews are requested (Gemini and the Engineering Lead). None
   has been received at the time of writing.

## Interpretations the author made

- "Wearer acts" means the physical button or the touch display, not the serial
  console.
- "Continue observing and communicating" means keep probing and reconfiguring
  the PCA9685, with outputs off, and keep reporting state.

## Open for the owner

#9: whether an ESP32 restart, or a bus outage that does not reset the chip,
should also wait for the wearer.

See `events.jsonl`. Events without a recorded time carry `"ts": null`.

## Follow-up

- Gemini approved `9d35445` as one of two required firmware reviews.
- PR #10 was merged at that revision. The author found no record of the second
  review on the pull request or in the ledger.
- A re-check by the author, requested by the owner, found two defects the author
  and that review had not caught. The fix was pushed shortly after the merge and
  so was not part of it; it is proposed separately and needs its own two reviews.
- The author's first summary to the owner said grip commands sent while limp did
  not run on resume. That was wrong for `9d35445`.
- #11 records a timing defect that predates this work.

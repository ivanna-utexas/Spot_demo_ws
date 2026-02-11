---
trigger: always_on
---

- All ros commands must be wrapped with a `./container cmd 'COMMAND'` or after running `./container shell`

- Don't spam ./container cmd ... repeatedly since every call spins up new processes/participants
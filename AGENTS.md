# Repository instructions

- This is a monorepo for the RoboMaster integration team.
- Keep each assignment under its own directory in `projects/`.
- Do not mix unrelated assignment changes in one commit or pull request.
- Create a feature branch instead of committing directly to `main`.
- Run the relevant project tests before committing when they are available.
- Never commit secrets, tokens, private keys, device credentials, generated build output, caches, or local logs.
- Do not force-push, delete branches, change repository visibility, or rewrite imported history without explicit approval.
- Preserve the imported histories under `projects/lab1/` and `projects/lab2/`.

## Code review rules

- Flag changes that commit generated ROS 2 directories such as `build/`, `install/`, or `log/`.
- Flag hard-coded credentials, host keys, access tokens, or machine-specific absolute paths.
- Check that changes stay within the intended project directory unless shared infrastructure is deliberately being updated.

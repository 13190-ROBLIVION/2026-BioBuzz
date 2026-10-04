# Contributing to the 13190-Roblivion implementation of the FTC SDK

## Setup
1. Get [git](https://git-scm.com/install/) and [github CLI](https://cli.github.com/). 
2. Clone repo: `git clone https://github.com/13190-ROBLIVION/2026-BioBuzz.git`
3. Cd into repo (wherever you put it; probably documents) `cd 2026-BioBuzz`
4. Open it in android studio

## Making changes
1. Make a branch `git switch -b yourfirstname/your-feature`
2. Make changes in andriod studio
3. Add all changes `git add .`
4. Commit changes and add a message in editor `git commit`
   * If it uses VIM, go ask an ai to make it use vscode
6. Push changes to remote `git push`
   * If it says you are out of date with remote: `git pull`
   * If it says there is doesn't know the remote branch do: `git config --global push.autoSetupRemote true` (Many people prefer this, so it is recommended to do it, or you could add -u origin/branch-name.
7. Create a pull request to main
  * Make sure all checks pass
  * Wait for someone to decide to merge ig

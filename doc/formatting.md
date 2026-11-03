# Java Formatting

TeamCode is formatted with the **Eclipse JDT formatter**, driven by
[Spotless](https://github.com/diffplug/spotless) and the profile in
[`config/eclipse/formatter.xml`](../config/eclipse/formatter.xml).

The style is the one the team already uses: **4-space indentation**, a
**120-column** limit, K&R braces, wrapped arguments indented one level with
the closing `)` on its own line, and **no blank line forced after a class's
opening brace**. Comment interiors are never touched.

This is *not* Palantir format and *not* google-java-format — both are
intentionally banned (see `config/checkstyle/checkstyle.xml`).

## Running it yourself

```bash
./gradlew :TeamCode:spotlessApply   # format TeamCode
./gradlew :TeamCode:spotlessCheck   # check only, fails if unformatted
```

Only `TeamCode` is formatted — `FtcRobotController` is upstream sample code
and is left alone (same policy as the Java Lint workflow).

## Editor setup

The same `config/eclipse/formatter.xml` drives every editor, so what you see
locally is exactly what CI produces.

### VS Code

Nothing to do — `.vscode/settings.json` in this repo already points the
"Extension Pack for Java" at the profile (`java.format.settings.url`), and
format-on-save is enabled for Java files. The Java language server formats
with the Eclipse engine natively, so no extra extension is needed.

### Android Studio / IntelliJ

The IDE's built-in formatter is IntelliJ's own engine, which is *not*
byte-compatible with the Eclipse profile. To get identical output, install
the free **"Adapter for Eclipse Code Formatter"** plugin (by Jeen Hwan Lee,
available from JetBrains Marketplace):

1. `Settings → Plugins → Marketplace` → search **Adapter for Eclipse Code
   Formatter** → install.
2. `Settings → Other Settings → Eclipse Code Formatter`.
3. Check **Use the Eclipse Code Formatter**.
4. Select **Eclipse XML profile file** and point it at
   `config/eclipse/formatter.xml` in this repo.
5. Pick the **BioBuzz** profile.

## What CI does

The **Java Format** workflow (`.github/workflows/java-format.yml`) runs on
every pull request:

1. formats `TeamCode` with `spotlessApply`,
2. runs the same Checkstyle rules as the Java Lint workflow **on the
   formatted result** — this validates the exact tree that is about to be
   pushed (pushes made with `GITHUB_TOKEN` never trigger other workflows,
   so the lint has to run in the same job),
3. commits and pushes the formatting changes back to the branch if there
   are any (`chore: auto-format Java code`).

It can also be run manually on any branch via *Run workflow*.

## Changing the style

Edit `config/eclipse/formatter.xml` (it is a plain Eclipse formatter
profile — every setting is a documented `org.eclipse.jdt.core.formatter.*`
key). Then run:

```bash
./gradlew :TeamCode:spotlessApply
```

and commit the result together with the profile change, so CI stays green.

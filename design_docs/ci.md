# CI

## What it does, at a glance

GitHub Actions runs on every pull request and exercises three Tier 1 checks against an upstream MoveIt Pro Docker image:

1. **`ensure-no-ssh-in-gitmodules`** — reproducibility guard. SSH (`git@`) submodule URLs break for CI (no SSH key on the runner) and for any contributor who isn't authenticated as the repo owner. The job greps `.gitmodules` and fails fast if any `git@` URL slips in.
2. **`validate_objectives`** — lints every behavior-tree XML in the workspace via `PickNikRobotics/moveit_pro_lint@v0.0.1`. Catches malformed BT nodes, missing ports, and other structural issues before they reach a real runtime. The job intentionally checks out the repo **without submodules** (`submodules: false`, set explicitly so a shared default can't re-enable it). The reasoning: the lint globs `**/*.xml` and attempts `xml.etree.parse` on every match. Well-formed non-BT XML (launch files, plugin descriptions, robot description) parses fine and is silently skipped because there's no `<BehaviorTree>` element — those don't cause failures. The actual failure mode is files that aren't *standalone-parseable* XML at all, e.g. MuJoCo `<include>` fragments some third-party submodules ship, which lack a root element. Our objectives all live in `src/picknik_009_ur5e_*/objectives/`, so this job has no use for submodule contents.
3. **`integration_test`** — pulls the prebuilt MoveIt Pro Docker image, layers the workspace on top, runs a full-workspace `colcon build` followed by `colcon test`. Packages under `src/external_dependencies/` (submodules) are built so they're available as link/exec dependencies but auto-skipped during `colcon test` (the reusable workflow handles that with `colcon list --base-paths src/external_dependencies --names-only | xargs --packages-skip`). We do not build MoveIt Pro itself from source; the upstream image is the contract.

   **Why full-workspace and not `--packages-up-to <config>`:** the C++ behavior packages (`shelf_stocking_behaviors`, `pcl_nn_filter`, `get_graspable_grocery_objects_from_masks3d`) are loaded at runtime via `pluginlib`, so they don't appear in any config package's `package.xml`. A `--packages-up-to picknik_009_ur5e_config_sim` build silently skips them, and `colcon test --packages-select picknik_009_ur5e_config_sim` would skip their gtests even if they'd been built. Those gtests are where most of the workspace's real test coverage lives — `test_gemini_response_step`, `test_publish_pipeline_step_status`, behavior plugin smoke tests, etc. Without them, this job mainly proves "the config packages install cleanly," which is a weaker signal than the build time implies. The trade is slower CI runs (full workspace build on `ubuntu-22.04` ≈ 10–20 min) for real test coverage.

`pre-commit` formatting is a separate workflow (`.github/workflows/format.yaml`) that predates this CI.

## Triggers — PR-only by design

The workflow runs on `pull_request` and `workflow_dispatch` (manual trigger from the GitHub Actions UI, useful for re-running against a different image tag without opening a PR). It does **not** run on `push: main` or on a schedule — those triggers were stripped from the template at adoption time because we don't want CI noise from merges or cron drift detection.

## Image pin — `image_ref`, not `image_tag`, and why

`ci.yaml` pins the MoveIt Pro image this workspace is developed against. The pin lives once, as the fallback in the `image_ref:` input expression — that's the source of truth.

**The input is `image_ref`, and switching back to `image_tag` will break the pull.** `image_tag` is a *version prefix* that the reusable workflow formats into the **Pro 9** image name, `picknikciuser/moveit-studio:<tag>-<distro>`. Pro 10 publishes to a **renamed Docker Hub repo** — `picknikciuser/moveit-pro` — and the old `moveit-studio` repo has no `10.x` tag at all, so no value of `image_tag` can reach a Pro 10 image. `image_ref` overrides the entire reference; its `{0}` is substituted with the matrix's `ros_distro`.

**Jazzy only.** `ros_distros` is set to `'["jazzy"]'` rather than the reusable workflow's default `'["humble", "jazzy"]'`. This workspace does not build on Humble — `<tl/expected.hpp>` exists only on Pro 10, so a Humble leg fails at compile time in `get_graspable_grocery_objects_from_masks3d`. That failure is a wrong-image signal, never a code regression; read it as "someone re-added humble to the matrix" or "the pin slipped back to a Pro 9 image".

**When to bump:**
- A newer Pro 10 patch release cuts → bump the version in `image_ref`.
- The workspace starts requiring a feature on a newer upstream tag → bump to that tag.
- Never pin to `main-jazzy` or any floating tag for CI. It would catch upstream breakage earlier, at the cost of CI going red without this repo changing — the wrong trade for a signal that gates merges.
- Keep the pin in step with the Pro 10 CLI that builds the workspace locally (see [`moveit_pro_10_build.md`](moveit_pro_10_build.md)); a CI image ahead of or behind the local CLI is how a green PR still fails to build on the bench.

## Image source — public Docker Hub, no privacy story needed

`picknikciuser/moveit-pro:*` on Docker Hub (`picknikciuser/moveit-studio:*` for Pro 9 and earlier) is the upstream-built, publicly-readable image. Customer code lives in this workspace and gets layered on top inside CI; nothing customer-specific is published. If that ever changes (e.g. a private image with bundled customer assets), the workflow would need a `docker/login-action` step and a private registry; right now it doesn't.

## Secrets

`MOVEIT_LICENSE_KEY` is an **organization-level** secret in `PickNikRoboticsServices`, inherited by this repo via the org secret access policy. The workflow references it as `${{ secrets.MOVEIT_LICENSE_KEY }}` — the same syntax as a repo-level secret, no special handling.

**Fork-PR caveat:** GitHub policy strips secrets from workflows triggered by `pull_request` from a fork. `integration_test` will fail with a missing-license error on such PRs. Internal contributors pushing branches directly to `shelf_stocking_ws` are unaffected. If outside contractors start opening fork-PRs, swap the trigger to `pull_request_target` (with the usual checkout-the-untrusted-ref caveats) or accept that they have to push branches into the repo.

## `ament_copyright` is disabled on the config packages

`picknik_009_ur5e_config`, `picknik_009_ur5e_config_sim`, and `picknik_009_ur5e_hw_config` no longer declare `ament_cmake_copyright` as a `<test_depend>` in their `package.xml`. Removing the test_depend (rather than fiddling with `set(<pkg>_FOUND TRUE)` in CMake) is the only mechanism in humble's `ament_lint_auto` that actually prevents the copyright hook from registering: the macro calls `find_package(<dep> QUIET)` unconditionally for every entry in `${PROJECT_NAME}_TEST_DEPENDS`, which sources the package's extras file and registers the lint hook regardless of any preset `_FOUND` variable. The widely-copied "set FOUND TRUE to disable" idiom is therefore a no-op in this version. Skip the dep in `package.xml` and the foreach never sees it.

**Why disabled, not patched:**
- This workspace has no copyright policy to enforce — packages are marked `<license>Proprietary</license>` and are not redistributed.
- The sim package's directory recursively includes the `YCB_Dataset` submodule (`mjcf/assets/ycb_dataset/scripts/`), whose Python files we do not own and cannot modify. `ament_copyright` *does* honor `AMENT_LINT_AUTO_FILE_EXCLUDE` via its lint hook, so path-based exclusion is technically possible, but the workspace has no proprietary-license template that ament_copyright would recognize on our own files anyway — the four `mjcf/assets/textures/*.py` generators and the launch/script Python files would still fail with "could not find copyright notice." Skipping the lint entirely is simpler than carrying both an exclude list and a custom license template.

**If you ever want copyright enforcement back,** add `<test_depend>ament_cmake_copyright</test_depend>` to the `package.xml` and either (a) define `AMENT_LINT_AUTO_FILE_EXCLUDE` to a list including `mjcf/assets/ycb_dataset` (and add Apache-2.0-style headers to the rest), or (b) register a custom license template via ament_copyright's CLI tooling. Neither is worth doing today.

The other auto-discovered linters (`ament_clang_format`, `ament_clang_tidy`, `ament_cmake_lint_cmake`) stay enabled. The config packages have effectively no C/C++ sources, so clang-format and clang-tidy are no-ops; `lint_cmake` runs against our short CMakeLists.txt files and is cheap.

## Lint pin — why `v0.0.1` and not newer

We pin `moveit_pro_lint@v0.0.1` deliberately. `v0.0.2` dropped support for the legacy `<BehaviorTree _description="...">` attribute and requires the newer `<Metadata description="..." />` form inside `<MetadataFields>`. Our objectives still use the `_description` attribute form (~140 files). Bumping to `v0.0.2` would fail validation on every existing objective until a migration pass converts them. When/if we run that migration, bump the lint pin in the same PR.

The lint also has no built-in exclude mechanism. Working around the lack of excludes by not fetching submodules (above) is the practical alternative.

## Reusable workflow — pin to a tag, never `main`

`integration_test` calls a reusable workflow from `PickNikRobotics/moveit_pro_ci`. Pin to a tagged version; `main` and `virtual-buffer` roll without notice and are the most common cause of "CI was green yesterday, red today, no commits in between."

## Disk is the binding constraint on a GitHub-hosted runner

`integration_test` runs inside a `container:` job, which means **no step can run before the
image is pulled** — the usual "free up 25 GB of preinstalled toolchains first" trick is
unavailable. The runner's disk therefore has to hold the Pro 10 image plus a full-workspace
build, and it only just does.

This is why `ci.yaml` passes an explicit `colcon_build_args` overriding the mixin list from
the workspace's own [`colcon-defaults.yaml`](../colcon-defaults.yaml). That file is tuned for
developer machines and enables `rel-with-deb-info` alongside `coverage-gcc` /
`coverage-pytest`. The debug symbols dominate — the workspace's `.o` files run north of a
gigabyte with them on, and the main behavior library alone is a couple hundred megabytes —
and nothing in CI consumes the coverage data, so it is pure cost here.

The failure it prevents is worth recognizing on sight, because it does **not** look like a
disk problem: `colcon build` succeeds, `colcon test` starts, and five to seven seconds later
the job dies with every remaining step left `pending` and no test output at all. The real
message only appears in the run's *annotations*, not the logs:

```
Unhandled exception. System.IO.IOException: No space left on device :
  '/home/runner/actions-runner/cached/<version>/_diag/Worker_<timestamp>-utc.log'
```

Two consequences to keep in mind when editing this:

- **`build-testing-on` must stay in the overridden mixin list.** Drop it and no test targets
  are built, so `colcon test` finds nothing to run — a green job that tested nothing, which
  is worse than the red one it replaces.
- **The headroom is thin, not generous.** The same job passed repeatedly under Pro 9 / Humble
  on this same runner; the slightly larger Pro 10 image is what pushed it over. Expect to
  revisit this the next time the image grows, and prefer trimming what CI builds over
  assuming a bigger runner will appear — see *Runner choice* below for why one won't.

## Runner choice

`ubuntu-22.04`, GitHub-hosted. The reusable workflow's default `studio_16_core_runner` is not available to repos in `PickNikRoboticsServices` — jobs targeting it queue for 24h and then cancel. Override only if a self-hosted runner is wired in for this repo specifically.

## What CI does NOT cover

- **Headless launch / objective smoke tests.** The template's `objective_smoke.yaml` (Tier 2) is not adopted. We don't currently exercise "does the workspace actually run an objective end-to-end" in CI. The shelf-stocking application is verified by hand on hardware.
- **Live hardware behavior.** All in-workspace packages (including `picknik_009_ur5e_hw_config`) are built and their declared tests run, but `colcon test` does not stand up a real ROS network or talk to any real device. Anything that requires an actual UR driver, EPick gripper, RealSense camera, or rosbridge handshake at runtime is verified by hand on the rig, not by CI. The behavior-package gtests use mocks/fakes for ROS interactions, which catches logic regressions but not driver/firmware/wiring issues.
- **GPU / CUDA paths.** The CI image is the CPU `jazzy` variant, not `jazzy-cuda13.2-cudnn9`. Any GPU-dependent perception step (e.g. SAM3 inference) is not exercised.
- **The Humble/Pro 9 stack.** The matrix is Jazzy-only by design (see above), so nothing here proves the workspace still builds against Pro 9. It doesn't, and isn't meant to — the Pro 9 line is frozen at the `v9-final` tag.

- **The shared Objectives' own checks.** `validate_objectives` does not fetch submodules, so the Objectives in `src/shelf_stocking` are linted by that repo's CI, not this one. Its C++ Behaviors are built and tested here, but only when the job fetches submodules: `src/shelf_stocking` is a private `PickNikRoboticsServices` repo, so this workflow needs a token that can read it.

## `markdown-link-check` checks relative links only, and why

The `Format` workflow (`pre-commit`) had been failing since June — on `main` as well as any
branch — with a failure mode that reads as a tool bug rather than a link problem.
`markdown-link-check` prints a file's `FILE:` header and then **nothing**: no link results, no
`N links checked`, no error message. The run then exits 1. Because an aborted file contributes
no results at all, it is invisible in the totals, so the summary shows every remaining link
healthy (`alive` + `ignored` reconciling exactly against the per-file counts, zero `dead`)
next to a failed hook. The run's annotations add nothing but `Process completed with exit code
1`.

**It is external-URL checking that aborts, and no single URL is at fault.** Removing every
external link from `README.md` makes it pass; removing any *one* of them does not; removing
only the relative links does not. It is load- and network-dependent, which is why it
reproduced on CI's shared egress long before it reproduced on a workstation. The
[`.markdown-link-check.json`](../.markdown-link-check.json) config therefore ignores
`^https?://` outright, and its `_comment` records the reasoning.

**What that gives up, and what it keeps.** External links rot slowly and a reader notices when
one 404s. Relative links between design docs rot *fast* and silently — every doc rename,
split, or deletion breaks a handful, and nothing else in the toolchain catches them. That is
the check worth having, and it still works: a link to a nonexistent doc is reported `[✖]` with
`ERROR: 1 dead links found!`. The `src/external_dependencies/` entry stays alongside, because
those relative paths point into submodules CI does not check out.

Two things to know if this resurfaces:

- **Do not read "0 dead links" as "the links are fine."** Compare the number of `FILE:`
  headers against `git ls-files '*.md' | wc -l`, then look for a header with no results
  beneath it. That is the aborted file.
- **Bisect with `pre-commit run markdown-link-check --files <file>`,** one file at a time: the
  hook is silent in aggregate but its per-file exit code is not. Running the CLI straight out
  of its pre-commit cache does *not* work — it resolves dependencies through the hook's own
  node environment and dies on a missing module, which looks like the same abort and is not.

The hook stays pinned to an old release deliberately: v3.12 and later are ESM-only and fail to
load under this repo's pinned `node` language version with `ERR_REQUIRE_ESM`, while v3.11.x
reproduces the abort rather than fixing it. v3.11.x is, however, the useful debugging tool —
it reproduces on a workstation what v3.10.3 only does in CI.

## Failure triage

| Failing job | Most likely cause | Where to look |
|---|---|---|
| `ensure-no-ssh-in-gitmodules` | Someone added a submodule with a `git@` URL | `.gitmodules`; flip to `https://` |
| `validate_objectives` — `not well-formed (invalid token)` | The lint is parsing a non-BT XML file as a BT (likely a submodule fragment) | Confirm `submodules: false` is still set on this job's `actions/checkout` (i.e. submodules were not fetched); this job validates our objectives only |
| `validate_objectives` — `Objective description not found` | A new objective is missing a description | Add `_description="..."` to the `<BehaviorTree>` element (matches the existing fleet) or `<Metadata description="..." />` inside `MetadataFields` |
| `validate_objectives` — other | A BT XML has missing `subcategory`, missing `MetadataFields`, deprecated `_subtreeOnly`, etc. | Lint message names file + assertion |
| `integration_test` at image-pull | The ref in `image_ref` doesn't exist on Docker Hub | Check it on hub.docker.com — and check the **repo** first: Pro 10 is `picknikciuser/moveit-pro`, Pro 9 was `picknikciuser/moveit-studio` |
| `integration_test` at `colcon build` — `fatal error: tl/expected.hpp: No such file or directory` | CI is on a Pro 9 image. That header only exists on Pro 10; the workspace moved off the `<tl_expected/expected.hpp>` shim deliberately | Fix the pin, not the include. Check `image_ref` and that `ros_distros` is still `'["jazzy"]'` |
| `integration_test` at `colcon build` for `moveit_studio_ur_pstop_manager` | `ur_dashboard_msgs` rosdep not installed in image | See "Dependencies come from the image build" in [`moveit_pro_10_build.md`](moveit_pro_10_build.md) for `moveit_studio_ur_pstop_manager` and `ur_dashboard_msgs`" |
| `integration_test` at `colcon build` — `fatal error: moveit_pro_behavior_interface/test/<header>.hpp: No such file or directory` | A test includes a header from the upstream package's `test/include/` subtree. That subtree is **not** exported by the prebuilt MoveIt Pro image (only the public `moveit_pro_behavior_interface/*.hpp` headers are installed, no `test/` subdirectory). Locally it builds because we compile moveit_pro from source. | Vendor a local copy of the helper into the workspace test rather than including the upstream `test/` header. See `src/shelf_stocking_behaviors/test/test_gemini_response_step.cpp` for the precedent (local `waitWhileRunning`). Re-check `docker run --rm --entrypoint bash picknikciuser/moveit-studio:<tag> -c 'ls /opt/overlay_ws/install/moveit_pro_behavior_interface/include/moveit_pro_behavior_interface/test/'` when bumping image tag — if the directory starts shipping, the local copies can be dropped. |
| `Format` fails with every link reported healthy | A file aborted mid-check; external-URL checking is the usual trigger | Count `FILE:` headers vs `git ls-files '*.md' \| wc -l` and find the header with no results — see *`markdown-link-check` checks relative links only* above |
| `integration_test` dies seconds into `colcon test` with no test output, remaining steps `pending` | Runner disk exhausted — read the run's **annotations**, not the logs, for `No space left on device` | Do not chase a test bug; it is disk. Check `colcon_build_args` still overrides the dev mixins — see *Disk is the binding constraint* above |
| `integration_test` at `colcon test` (`ament_copyright`) | A package added `ament_cmake_copyright` back as a `<test_depend>` in `package.xml` | Remove the test_depend; see "`ament_copyright` is disabled..." above |
| `integration_test` at `colcon test` (`shelf_stocking_behaviors`, `pcl_nn_filter`, `get_graspable_grocery_objects_from_masks3d`) | gtest in a behavior package failing | Reproduce locally: `colcon build --packages-up-to <pkg> && colcon test --packages-select <pkg>` in a clean dev container, then `colcon test-result --verbose` |
| `integration_test` at `colcon test` (other) | A real unit test is broken | Reproduce locally with the dev-container build/test commands |
| `integration_test` missing license | Org secret access policy doesn't include this repo, OR PR is from a fork | Org Settings → Secrets → confirm `shelf_stocking_ws` is in scope |

## ZED packages are skipped

The `zed-ros2-wrapper` and `zed-ros2-interfaces` submodules need the Stereolabs ZED SDK and CUDA, which the CI image does not have. `colcon_build_args` in `ci.yaml` skips their packages. This works only because no workspace package declares a dependency on them: colcon refuses to skip a package that another package depends on. So the shelf_cam launch files reach `zed_wrapper` at run time without an `exec_depend` ([`zed_shelf_cam.md`](zed_shelf_cam.md)). Keep the skip list in step with the packages in those submodules.

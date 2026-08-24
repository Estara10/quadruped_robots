# Artifact Management

`manifest.yaml` is the tracked inventory for model and external binary artifacts needed by this project.

- `normal_git`: small deployment artifact committed with source code.
- `external`: large checkpoint/model stored outside ordinary Git; a cryptographic hash and source record are mandatory.
- `unknown` provenance remains explicit until the responsible task resolves it.

Local restores may be placed under `artifacts/local/`, which is ignored. Never replace a deployment artifact without updating the manifest and validation evidence.

P1-01 model entries distinguish the immutable deployed artifact from candidate source files. `repository_introduced_commit` or `repository_replaced_commit` records when a binary entered this repository; it is not a training commit. A candidate filename, archive root or co-located export script is indirect evidence only and never closes provenance without a source hash or reproducible export.

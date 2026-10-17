# License and provenance inventory

Reviewed 2026-10-17 (Asia/Seoul). The user has not selected a license for original PROJECT AETHER source. No MIT, Apache, GPL, or other project-wide license is imposed by this delivery. Choose a source license before public distribution. This inventory is an engineering record, not legal certification.

| Item | Current scope | License action |
| --- | --- | --- |
| Original project source, synthetic coefficients and procedural test data | Written for this project; no claim that coefficients describe a validated real aircraft | Source license remains unspecified; retain the synthetic-data labels |
| Zig portable toolchain | Build dependency, selected version 0.17.0; outside the application source | Preserve the distribution's license and third-party notices if redistributing the toolchain |
| C++ standard library / compiler runtime / Windows target support | Components may be linked or incorporated by the actual build | Copy the applicable LLVM/libc++/compiler-runtime/MinGW notices from the exact compiler distribution and record linked dependencies; do not label the whole binary as covered solely by Zig's license |
| Unreal Engine | Not installed or bundled | Future use is governed by Epic's EULA, not an open-source grant |
| Cesium / map datasets / imagery / aircraft assets / fonts / audio | None adopted or bundled | See candidate review; no permission is inferred from a candidate listing |

The exact compiler notice set and executable dependency inventory must accompany a distributable binary. A compiler's presence is not proof that every listed runtime component was linked; confirm against the selected build target and resulting executable. The original toolchain distribution is the source of truth for its bundled notices. [Zig distribution metadata](https://ziglang.org/download/index.json), [Zig overview](https://ziglang.org/learn/overview/).

Unreal's current agreement separates royalty products, royalty-free uses, seat requirements and exceptions. A commercial interactive product must be classified under the actual EULA, including distribution notice and reporting duties where applicable. Engine content and marketplace assets may carry separate licenses. No Epic account, EULA acceptance, or installation was performed for this project. [Unreal EULA](https://www.unrealengine.com/eula/unreal), [licensing overview](https://www.unrealengine.com/license).

Cesium for Unreal's Apache 2.0 code license does not grant rights to ion-hosted or third-party content. Data attribution, cache/offline rights and service terms remain separate. [Plugin FAQ](https://cesium.com/learn/unreal/unreal-faq/), [ion terms](https://cesium.com/legal/terms-of-service).

For each future imported asset, retain its source, exact license, author/credit, version, checksum, modification record and distribution constraints. Product-specific Copernicus notices must be copied from the applicable license when data is adopted; they are not reproduced here because no dataset is bundled. See [MAP_AND_GEO_DATA.md](MAP_AND_GEO_DATA.md) for the candidate review.

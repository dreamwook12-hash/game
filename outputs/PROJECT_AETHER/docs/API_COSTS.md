# External service costs

Checked 2026-10-17 (Asia/Seoul). **No paid API, cloud account, subscription or API key was enabled. Map-service spend incurred by this implementation: USD 0.** Reading public documentation and obtaining a local compiler are separate from map API usage. The application has no adopted online scenery provider.

The following are dated reference prices, not a purchase recommendation or production budget. Taxes, regional pricing, negotiated agreements, quota aggregation and later changes can alter charges.

| Candidate | Published reference | Decision |
| --- | --- | --- |
| Google 2D Map Tiles | Global pay-as-you-go list: 100,000 monthly free events; next tier USD 0.60 per 1,000 events, through 1,000,000 monthly events | Not enabled |
| Google Photorealistic 3D Tiles | 1,000 monthly free events; next tier USD 6.00 per 1,000 events, through 100,000 monthly events | Not enabled |
| Cesium ion | Community is for eligible personal/evaluation use; Commercial individual displayed at USD 149/month and team at USD 524/month | Not subscribed; distributed solutions need integration licensing review |
| Self-hosted open datasets | Data licensing may be free, but storage, processing, traffic and updates still consume resources | No hosting provisioned |

Google figures come from the [official global price table](https://developers.google.com/maps/billing-and-pricing/pricing). A 3D root request and renderer tile requests have different accounting; do not estimate cost by multiplying rendered tile count by the root-request price. Billing and credentials are required even where free usage applies. [Usage/billing definitions](https://developers.google.com/maps/documentation/tile/usage-and-billing).

Cesium plan prices and limits do not themselves authorize integration in software offered outside the organization. Its pricing page directs such use to separate integration pricing/licensing. [Current plans](https://cesium.com/platform/cesium-ion/pricing/).

Before any paid service activation, the user's explicit approval is required by the project request. Present a concrete provider, permitted use, expected monthly requests/bytes, quoted price, maximum budget and offline behavior. A future implementation should use provider quotas and a local request budget; alerts alone do not enforce a spending ceiling. No billing alerts, quotas or API controls were configured during this work.

# Eight failure cases: completed contact-timeconstant result

All eight predeclared failure cases completed. Zero pass the unchanged supplied
slide metric and zero complete the physical slide diagnostic. Subsequent control results: both 176 and 225 lose their prior completion. This setting is not adopted.

| UID | Baseline final tilt | Probe final tilt | Probe final center distance | Probe physical sequence |
|---|---:|---:|---:|---|
| 113 | 90.0 deg | 90.0 deg | 209.4 mm | no_supported_push_to_contact |
| 130 | 90.0 deg | 90.0 deg | 235.4 mm | no_supported_release |
| 121 | 90.0 deg | 0.0 deg | 95.7 mm | no_supported_push_to_contact |
| 173 | 90.0 deg | 0.0 deg | 115.5 mm | no_supported_push_to_contact |
| 211 | 90.0 deg | 0.0 deg | 241.1 mm | no_supported_push_to_contact |
| 212 | 90.1 deg | 0.0 deg | 379.3 mm | no_supported_release |
| 197 | 90.0 deg | 90.0 deg | 265.5 mm | no_supported_release |
| 204 | 90.0 deg | 90.0 deg | 161.2 mm | no_supported_release |

The experiment reduces some in-grasp losses but fails to recover the requested
complete sequence in this sample. It is not evidence that further stiffening
would help, and no wider parameter sweep is justified by these outcomes alone.
Source commands remain byte-identical to baseline; all modified contact settings
were checked in the built world. Initial poses remain fitted estimates.
Trial 113 additionally has a timestamp-matched real/sim landing review.

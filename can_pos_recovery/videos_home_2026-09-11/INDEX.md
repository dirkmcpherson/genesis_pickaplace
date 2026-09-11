# The 27 `home` tapes under the calibrated Ladder N predicate (rendered 2026-09-11, ladder=nested_ramp)

Every tape here pays `home` (settled arrival after a far-side slide) under the calibrated constants: FARSIDE reach 0.10 m, 60° cone, set-down latch, gain ≥ 1 cm; `far_release` off. These are ALL the tapes that pay it: 13/74 human, 14/72 machine — exactly the sim-slide tapes, no other tape in either set. What to check by eye: (1) PLACE lights when the can is set down; (2) FAR lights only after that, with the gripper on the goal-opposite side of the can; (3) SLIDE2 lights only once the can has come to rest after the release AND then moves goalward with the gripper behind it (never during the release itself); (4) HOME lights when the can is at rest within 0.081 m of the goal, upright, not in hand. The reward line shows the nested_ramp pay (max 9) with the CODE DEFAULT ramp span of 0.10 m — which is why the human slides read 7.2–8.3 of 9; amendment (aa) registers 0.05 m for the batch, not yet applied in the code at render time.

| set | uid | clip | decisions | picked | placed_v2 | farside | slide_event | home | ramp reward (of 9) | end reason | settle nested |
|---|---|---|---|---|---|---|---|---|---|---|---|
| human | 232 | `human_232_slide.mp4` | 255 | d70 | d121 | d190 | d194 | d201 | 7.558447265790775 | home | True |
| human | 233 | `human_233_slide.mp4` | 260 | d69 | d174 | d199 | d207 | d212 | 7.417425142633988 | home | True |
| human | 237 | `human_237_slide.mp4` | 233 | d89 | d155 | d175 | d182 | d183 | 7.210419106530442 | home | True |
| human | 247 | `human_247_slide.mp4` | 461 | d193 | d347 | d405 | d406 | d425 | 8.269565137731753 | home | True |
| human | 251 | `human_251_slide.mp4` | 421 | d189 | d298 | d356 | d357 | d362 | 7.546992152394495 | home | True |
| human | 256 | `human_256_slide.mp4` | 581 | d174 | d474 | d490 | d507 | d515 | 7.7822515066344735 | home | True |
| human | 259 | `human_259_slide.mp4` | 330 | d112 | d174 | d204 | d248 | d266 | 8.271010401545004 | home | True |
| human | 273 | `human_273_slide.mp4` | 392 | d124 | d218 | d225 | d284 | d300 | 8.159136464816465 | home | True |
| human | 275 | `human_275_slide.mp4` | 387 | d132 | d246 | d260 | d318 | d332 | 8.238853265320696 | home | True |
| human | 302 | `human_302_slide.mp4` | 318 | d167 | d241 | d244 | d273 | d280 | 7.483017309150257 | home | True |
| human | 304 | `human_304_slide.mp4` | 548 | d126 | d252 | d274 | d484 | d497 | 7.681964015405794 | home | True |
| human | 316 | `human_316_slide.mp4` | 500 | d178 | d291 | d314 | d420 | d468 | 7.979463203683638 | home | True |
| human | 317 | `human_317_slide.mp4` | 390 | d112 | d191 | d215 | d282 | d299 | 7.865059490582654 | home | True |
| machine | 263 | `machine_263_slide.mp4` | 600 | d113 | d277 | d397 | d402 | d430 | 8.428516005033126 | home | True |
| machine | 233 | `machine_233_slide.mp4` | 356 | d86 | d173 | d199 | d206 | d216 | 7.591519829713487 | home | True |
| machine | 255 | `machine_255_slide.mp4` | 600 | d96 | d176 | d184 | d223 | d232 | 7.292986958433729 | home | True |
| machine | 265 | `machine_265_slide.mp4` | 600 | d159 | d268 | d294 | d344 | d348 | 7.485368824359019 | home | True |
| machine | 302 | `machine_302_slide.mp4` | 337 | d143 | d228 | d236 | d269 | d276 | 7.263654769144503 | home | True |
| machine | 305 | `machine_305_slide.mp4` | 425 | d165 | d253 | d279 | d286 | d295 | 7.527081644168055 | home | True |
| machine | 259 | `machine_259_slide.mp4` | 600 | d115 | d219 | d228 | d286 | d298 | 8.264282620757934 | home | True |
| machine | 281 | `machine_281_slide.mp4` | 600 | d104 | d197 | d205 | d251 | d263 | 8.02514061837718 | home | True |
| machine | 242 | `machine_242_slide.mp4` | 600 | d98 | d186 | d219 | d286 | d319 | 8.828711011591503 | home | True |
| machine | 251 | `machine_251_slide.mp4` | 600 | d118 | d210 | d217 | d262 | d281 | 8.306900563763316 | home | True |
| machine | 321 | `machine_321_slide.mp4` | 600 | d147 | d215 | d222 | d247 | d261 | 7.684267167376676 | home | True |
| machine | 243 | `machine_243_slide.mp4` | 600 | d77 | d154 | d165 | d199 | d211 | 7.596787530379814 | home | True |
| machine | 262 | `machine_262_slide.mp4` | 600 | d180 | d288 | d432 | d471 | d480 | 8.415665206801073 | home | True |
| machine | 311 | `machine_311_slide.mp4` | 600 | d84 | d214 | d214 | d220 | d223 | 7.253800594648116 | home | True |

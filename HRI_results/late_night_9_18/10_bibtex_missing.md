# Item 10 — BibTeX for the code and methods we used (R2Dreamer, lerobot, and everything else missing)

**Status: done.** One paste-ready block for `sample-base.bib` (the only file `main.tex` loads: `\bibliography{sample-base}`).
Every entry carries a "verified from" line. Entries marked **USER MUST CHECK** were written from memory or from a
secondary source and were not confirmed against a primary page tonight. Existing entries that should be *replaced*
are given with the same key so the text does not change; new keys are given for things the text does not cite yet.
Three format cautions first:

- `sample-base.bib` defines `dreamerv3torch2024` **twice** (lines near "RLC main.bib" and near the end). BibTeX prints
  `Repeated entry` and skips the second; delete one copy (verified by running BibTeX on a copy).
- `@software{…}` entries (`Genesis`, `zakka2021ibc`, `torchvision2016`) are not a type ACM-Reference-Format.bst knows;
  it typesets the title as an italic *booktitle* and drops the `url`/`version` fields (verified: `t_sw.bbl` in the
  scratchpad `toctest/`). Use `@misc` with `howpublished={\url{…}}` as below.
- Keep keys stable: the text cites `morihira2026r2d` (not the upstream README's `morihira2026rdreamer`), `cadene2024lerobot`,
  `dreamerv3torch2024`, `Genesis`, `zakka2021ibc`, `hafner2023mastering`, `chi2024diffusionpolicy`, `ball2023efficient`.

## Paste-ready block (append to / replace in `sample-base.bib`)

```bibtex
%% ---------------------------------------------------------------- code we used
%% R2Dreamer paper (cited as morihira2026r2d in sections/03-method.tex).
%% verified from: ~/workspace/r2dreamer/17677_R2_Dreamer_Redundancy_Re.pdf p.1 ("Published as a conference paper at
%% ICLR 2026"; authors/affiliations), arXiv abs/2603.18202 (v2 2026-03-20), upstream README citation block
%% (https://github.com/NM512/r2dreamer). OpenReview id Je2QqXrcQq (page behind a bot check tonight; id from the README).
@inproceedings{morihira2026r2d,
  title     = {{R2-Dreamer}: Redundancy-Reduced World Models without Decoders or Augmentation},
  author    = {Morihira, Naoki and Nahar, Amal and Bharadwaj, Kartik and Kato, Yasuhiro and Hayashi, Akinobu and Harada, Tatsuya},
  booktitle = {The Fourteenth International Conference on Learning Representations (ICLR)},
  year      = {2026},
  eprint    = {2603.18202},
  archivePrefix = {arXiv},
  url       = {https://openreview.net/forum?id=Je2QqXrcQq}
}

%% R2Dreamer code: upstream implementation + our fork.
%% verified from: upstream README (NM512/r2dreamer); fork remote from `git -C ~/workspace/r2dreamer remote -v`
%% (dirkmcpherson/r2dreamer). USER MUST CHECK which commit to cite: local HEAD 7971a00; the cluster tree of record for
%% the pixel runs is $W/r2dreamer_px @ 0b1b9d8 (CLAUDE.md, PX_CLUSTER_DEPLOY_2026-09-13.md).
@misc{r2dreamer_code,
  author       = {Morihira, Naoki},
  title        = {{r2dreamer}: Implementation of {R2-Dreamer}},
  year         = {2026},
  howpublished = {\url{https://github.com/NM512/r2dreamer}},
  note         = {Fork with the Genesis restocking environment, pixel configuration and \texttt{rep\_loss=dreamer} variant: \url{https://github.com/dirkmcpherson/r2dreamer}}
}

%% LeRobot (Diffusion Policy implementation). REPLACES the existing cadene2024lerobot: the author list in the
%% current README has 16 names; the bib has 6.
%% verified from: ~/workspace/lerobot/README.md "## Citation" (verbatim). Fork/branch of record from
%% cluster/install_lerobot.sh: dirkmcpherson/lerobot, branch genesis-fixes = lerobot 0.4.5 + image_writer mkdir fix.
@misc{cadene2024lerobot,
  author       = {Cadene, Remi and Alibert, Simon and Soare, Alexander and Gallouedec, Quentin and Zouitine, Adil and Palma, Steven and Kooijmans, Pepijn and Aractingi, Michel and Shukor, Mustafa and Aubakirova, Dana and Russi, Martino and Capuano, Francesco and Pascal, Caroline and Choghari, Jade and Moss, Jess and Wolf, Thomas},
  title        = {{LeRobot}: State-of-the-art Machine Learning for Real-World Robotics in {PyTorch}},
  howpublished = {\url{https://github.com/huggingface/lerobot}},
  year         = {2024},
  note         = {Version 0.4.5; fork \url{https://github.com/dirkmcpherson/lerobot} (branch \texttt{genesis-fixes})}
}

%% dreamerv3-torch (the DfD/IBC/VMAIL chassis for PinPad4/PushT). KEEP ONE copy of the existing entry; this is it
%% with the fork added. verified from: existing sample-base.bib entry; fork remote from
%% `git -C ~/workspace/dreamerv3-torch/dreamerv3-torch remote -v` (dirkmcpherson/dreamerv3-torch). The upstream
%% README has no citation block, so the author/commit fields are the paper's own — USER MUST CHECK the commit.
@misc{dreamerv3torch2024,
  author       = {NM512},
  title        = {{dreamerv3-torch}: A {PyTorch} implementation of {DreamerV3}},
  year         = {2024},
  howpublished = {\url{https://github.com/NM512/dreamerv3-torch}},
  note         = {Commit a27711a; fork \url{https://github.com/dirkmcpherson/dreamerv3-torch}}
}

%% Genesis simulator. Fields verified verbatim against ~/workspace/Genesis/README.md "## Citation"; type changed
%% @software -> @misc so ACM-Reference-Format prints the URL. Version from `git describe`: v0.2.1-270-gf41427d
%% (0.2.1 + local headless-render patch; CLAUDE.md: "genesis-world 0.2.1 editable").
@misc{Genesis,
  author       = {{Genesis Authors}},
  title        = {Genesis: A Universal and Generative Physics Engine for Robotics and Beyond},
  month        = dec,
  year         = {2024},
  howpublished = {\url{https://github.com/Genesis-Embodied-AI/Genesis}},
  note         = {Version 0.2.1}
}

%% Stable-Baselines3 (RLPD is built on SB3's SAC: baselines/rl/rlpd_sac.py imports stable_baselines3.SAC,
%% SACPolicy, polyak_update; rlpd_pixel.py imports BaseFeaturesExtractor, sac.policies.Actor).
%% verified from: https://github.com/DLR-RM/stable-baselines3/blob/master/CITATION.bib (verbatim) and
%% https://jmlr.org/papers/v22/20-1364.html. Version 2.8.0 in .venv-eval (`python -c "import stable_baselines3"`);
%% USER MUST CHECK the cluster env's version.
@article{stable-baselines3,
  author  = {Antonin Raffin and Ashley Hill and Adam Gleave and Anssi Kanervisto and Maximilian Ernestus and Noah Dormann},
  title   = {Stable-Baselines3: Reliable Reinforcement Learning Implementations},
  journal = {Journal of Machine Learning Research},
  year    = {2021},
  volume  = {22},
  number  = {268},
  pages   = {1--8},
  url     = {https://jmlr.org/papers/v22/20-1364.html}
}

%% IBC implementation. Fields verified verbatim against https://github.com/kevinzakka/ibc README (raw); type
%% changed @software -> @misc for the ACM bst.
@misc{zakka2021ibc,
  author       = {Zakka, Kevin},
  title        = {A {PyTorch} Implementation of Implicit Behavioral Cloning},
  year         = {2021},
  month        = oct,
  howpublished = {\url{https://github.com/kevinzakka/ibc}},
  note         = {Version 0.0.1}
}

%% TorchVision (ImageNet-pretrained ResNet18 for DP). Existing entry is the official one; only the type changes.
@misc{torchvision2016,
  author       = {{TorchVision maintainers and contributors}},
  title        = {{TorchVision}: {PyTorch}'s Computer Vision library},
  year         = {2016},
  howpublished = {\url{https://github.com/pytorch/vision}}
}

%% ---------------------------------------------------------------- methods
%% RLPD (cited as ball2023efficient; KEY IS MISSING from the bib today).
%% verified from: https://proceedings.mlr.press/v202/ball23a.html (authors, volume 202, pages 1577-1594, editors).
@inproceedings{ball2023efficient,
  title     = {Efficient Online Reinforcement Learning with Offline Data},
  author    = {Ball, Philip J. and Smith, Laura and Kostrikov, Ilya and Levine, Sergey},
  booktitle = {Proceedings of the 40th International Conference on Machine Learning},
  series    = {Proceedings of Machine Learning Research},
  volume    = {202},
  pages     = {1577--1594},
  year      = {2023},
  publisher = {PMLR}
}

%% SAC (for the \todo{cite} after "like SAC" in sections/03-method.tex).
%% verified from: https://proceedings.mlr.press/v80/haarnoja18b.html (PMLR 80:1861-1870).
@inproceedings{haarnoja2018soft,
  title     = {Soft Actor-Critic: Off-Policy Maximum Entropy Deep Reinforcement Learning with a Stochastic Actor},
  author    = {Haarnoja, Tuomas and Zhou, Aurick and Abbeel, Pieter and Levine, Sergey},
  booktitle = {Proceedings of the 35th International Conference on Machine Learning},
  series    = {Proceedings of Machine Learning Research},
  volume    = {80},
  pages     = {1861--1870},
  year      = {2018},
  publisher = {PMLR}
}

%% DrQ-v2 (the RLPD pixel encoder: baselines/rl/rlpd_pixel.py "DrQ-v2 conv stack ... critic owns the encoder").
%% verified from: https://github.com/facebookresearch/drqv2 README (arXiv form) + dblp conf/iclr/YaratsFLP22 (ICLR 2022).
@inproceedings{yarats2022drqv2,
  title     = {Mastering Visual Continuous Control: Improved Data-Augmented Reinforcement Learning},
  author    = {Yarats, Denis and Fergus, Rob and Lazaric, Alessandro and Pinto, Lerrel},
  booktitle = {International Conference on Learning Representations (ICLR)},
  year      = {2022},
  url       = {https://openreview.net/forum?id=_SJ-_yyes8}
}

%% DrQ (the random-shift augmentation: rlpd_pixel.py SHIFT_PAD = 4; r2dreamer image_aug shift4).
%% verified from: https://github.com/denisyarats/drq README (verbatim). NB first author is Yarats, not Kostrikov.
@inproceedings{yarats2021image,
  title     = {Image Augmentation Is All You Need: Regularizing Deep Reinforcement Learning from Pixels},
  author    = {Yarats, Denis and Kostrikov, Ilya and Fergus, Rob},
  booktitle = {International Conference on Learning Representations (ICLR)},
  year      = {2021},
  url       = {https://openreview.net/forum?id=GY6-6sTvGaf}
}

%% DreamerV3, journal version. The text cites hafner2023mastering (arXiv). Recommended: cite the Nature paper —
%% either replace the arXiv entry under the SAME key (no text edit) or add this and switch the two \cite sites
%% (sections/02-related-work.tex, sections/03-method.tex).
%% verified from: Crossref 10.1038/s41586-025-08744-2 (Nature 640(8059):647-653, 2025).
@article{hafner2025mastering,
  title   = {Mastering diverse control tasks through world models},
  author  = {Hafner, Danijar and Pasukonis, Jurgis and Ba, Jimmy and Lillicrap, Timothy},
  journal = {Nature},
  volume  = {640},
  number  = {8059},
  pages   = {647--653},
  year    = {2025},
  doi     = {10.1038/s41586-025-08744-2}
}

%% TD-MPC2 (for \todo{cite TDMPC2 and R2Dreamer} in sections/05-conclusion.tex).
%% verified from: https://github.com/nicklashansen/tdmpc2 README citation block (verbatim) + iclr.cc/virtual/2024/poster/18722.
@inproceedings{hansen2024tdmpc2,
  title     = {{TD-MPC2}: Scalable, Robust World Models for Continuous Control},
  author    = {Hansen, Nicklas and Su, Hao and Wang, Xiaolong},
  booktitle = {International Conference on Learning Representations (ICLR)},
  year      = {2024}
}

%% Diffusion Policy. REPLACES chi2024diffusionpolicy (existing entry has no volume/pages/doi).
%% verified from: Crossref 10.1177/02783649241273668 — IJRR 44(10-11):1684-1704; online 2024-10-11, print issue 2025.
%% USER MUST CHECK which year to print (the key says 2024; the issue is 2025).
@article{chi2024diffusionpolicy,
  title   = {Diffusion Policy: Visuomotor Policy Learning via Action Diffusion},
  author  = {Chi, Cheng and Xu, Zhenjia and Feng, Siyuan and Cousineau, Eric and Du, Yilun and Burchfiel, Benjamin and Tedrake, Russ and Song, Shuran},
  journal = {The International Journal of Robotics Research},
  volume  = {44},
  number  = {10--11},
  pages   = {1684--1704},
  year    = {2025},
  doi     = {10.1177/02783649241273668}
}
%% RSS 2023 conference version, if the conference paper is preferred.
%% verified from: Crossref 10.15607/RSS.2023.XIX.026 (author order differs from the IJRR version; Tedrake absent).
@inproceedings{chi2023diffusion,
  title     = {Diffusion Policy: Visuomotor Policy Learning via Action Diffusion},
  author    = {Chi, Cheng and Feng, Siyuan and Du, Yilun and Xu, Zhenjia and Cousineau, Eric and Burchfiel, Benjamin and Song, Shuran},
  booktitle = {Proceedings of Robotics: Science and Systems (RSS)},
  year      = {2023},
  doi       = {10.15607/RSS.2023.XIX.026}
}

%% robomimic: NOT needed — no section cites it and tables/robomimic.tex is not \input anywhere
%% (grep 'input{tables' over sections/appendix/main). If it comes back, the robomimic paper is mandlekar2021matters;
%% upgrade that entry to the CoRL version:
%% verified from: https://proceedings.mlr.press/v164/mandlekar22a.html (verbatim BibTeX, key renamed to the paper's).
@inproceedings{mandlekar2021matters,
  title     = {What Matters in Learning from Offline Human Demonstrations for Robot Manipulation},
  author    = {Mandlekar, Ajay and Xu, Danfei and Wong, Josiah and Nasiriany, Soroush and Wang, Chen and Kulkarni, Rohun and Fei-Fei, Li and Savarese, Silvio and Zhu, Yuke and Mart{\'\i}n-Mart{\'\i}n, Roberto},
  booktitle = {Proceedings of the 5th Conference on Robot Learning},
  series    = {Proceedings of Machine Learning Research},
  volume    = {164},
  pages     = {1678--1690},
  year      = {2022},
  publisher = {PMLR}
}

%% ---------------------------------------------------------------- hardware / middleware
%% Kinova Gen3 lite (sections/01-introduction.tex fig caption, sections/04-evaluation.tex:67).
%% verified from: https://www.kinovarobotics.com/product/gen3-lite-robots (fetched; page heading "Discover our Gen3
%% lite robot"). USER MUST CHECK the access date.
@misc{kinova_gen3lite,
  author       = {{Kinova Inc.}},
  title        = {{Gen3 lite} robot},
  howpublished = {\url{https://www.kinovarobotics.com/product/gen3-lite-robots}},
  year         = {2024},
  note         = {6-DoF arm with two-finger gripper; accessed 2026-09-17}
}

%% ROS (sections/04-evaluation.tex:68 "recorded ... from ROS").
%% verified from: web search (Stanford AI Lab PDF ai.stanford.edu/~mquigley/papers/icra2009-ros.pdf; BibSonomy entry).
%% The volume/number/page values (3, 3.2, 5) are the conventional ones for this workshop paper — USER MUST CHECK.
@inproceedings{quigley2009ros,
  title     = {{ROS}: an open-source Robot Operating System},
  author    = {Quigley, Morgan and Conley, Ken and Gerkey, Brian and Faust, Josh and Foote, Tully and Leibs, Jeremy and Wheeler, Rob and Ng, Andrew Y.},
  booktitle = {ICRA Workshop on Open Source Software},
  volume    = {3},
  number    = {3.2},
  pages     = {5},
  year      = {2009},
  address   = {Kobe, Japan}
}

%% ---------------------------------------------------------------- statistics (appendix cites rouder2009; KEY MISSING)
%% verified from: Springer 10.3758/PBR.16.2.225 (Psychonomic Bulletin & Review 16(2):225-237, 2009).
@article{rouder2009,
  title   = {Bayesian $t$ tests for accepting and rejecting the null hypothesis},
  author  = {Rouder, Jeffrey N. and Speckman, Paul L. and Sun, Dongchu and Morey, Richard D. and Iverson, Geoffrey},
  journal = {Psychonomic Bulletin \& Review},
  volume  = {16},
  number  = {2},
  pages   = {225--237},
  year    = {2009},
  doi     = {10.3758/PBR.16.2.225}
}
```

## Optional upgrades of existing arXiv-only entries (same keys; not required)

- `hansen2022modem` → ICLR 2023 (web search: "published at ICLR 2023"; arXiv 2212.05698). Booktitle
  `International Conference on Learning Representations (ICLR)`, year 2023. Verified only via search — USER MUST CHECK.
- `hansen2022temporal` (TD-MPC) → ICML 2022, PMLR 162 (from memory — USER MUST CHECK pages).
- `lee2024behavior` (VQ-BeT) → ICML 2024; `zhao2024aloha` → CoRL 2024; `luo2024precise` (HIL-SERL) → RSS 2025;
  `zhao2023learning` (ACT) → RSS 2023. All from memory — USER MUST CHECK before changing.

## Discrepancies

1. `cadene2024lerobot` in the bib lists 6 authors; the fork's README (`~/workspace/lerobot/README.md:141`) lists 16.
   The 6-name version is the 2024 form; either is a legitimate snapshot, but the README of the code we ran is the 16-name
   one.
2. The paper says the lerobot fork is "the user's fork" (`cluster/install_lerobot.sh:10`, branch `genesis-fixes`), but
   `~/workspace/lerobot` is checked out at `d324ffe8` (2026-03-05, an upstream CI commit) with three remotes
   (`origin`=dirkmcpherson, `upstream`=huggingface, `alex`=alexander-soare). The bib note above names the branch of
   record, not the local checkout.
3. `dreamerv3torch2024` `commit = a27711a` / `note = Accessed 2024-01-04` cannot be verified from either README
   (neither the upstream nor the fork README has a citation block); the fork HEAD is `1ec0641` on branch
   `confound_test` (`git -C ~/workspace/dreamerv3-torch/dreamerv3-torch log -1`).
4. `Genesis` version: `git describe` → `v0.2.1-270-gf41427d`, i.e. 0.2.1 plus 270 upstream commits plus one local
   patch (headless render). The paper/appendix says "version 0.2.1" — say "0.2.1 (development snapshot)" or pin the hash.
5. `hafner2023mastering` (arXiv 2023) is cited where the Nature 2025 paper now exists — see the entry above.

## \llm notes inserted

- `sections/03-method.tex`, anchor `(like SAC\todo{cite})` — appended `\llm{Item 10/16: propose \cite{haarnoja2018soft} (PMLR 80:1861--1870, verified); BibTeX in HRI_results/late_night_9_18/10_bibtex_missing.md.}`
- `sections/05-conclusion.tex`, anchor `\todo{cite TDMPC2 and R2Dreamer}` — appended `\llm{Item 10/16: propose \cite{hansen2024tdmpc2, morihira2026r2d}; both BibTeX entries (morihira2026r2d is MISSING from sample-base.bib today) in HRI_results/late_night_9_18/10_bibtex_missing.md.}`
- (the other `\todo{cite…}` sites are covered by item 16's notes, listed in `16_citation_check.md`)

## Sources

- `~/workspace/r2dreamer/17677_R2_Dreamer_Redundancy_Re.pdf` (page 1), `~/workspace/r2dreamer/README.md` (Citation block), `git -C ~/workspace/r2dreamer remote -v`
- `~/workspace/lerobot/README.md` §Citation; `git -C ~/workspace/lerobot remote -v`, `log -1`; `cluster/install_lerobot.sh`
- `~/workspace/dreamerv3-torch/dreamerv3-torch/` (`git remote -v`, `log -1`, README grep — no citation block)
- `~/workspace/Genesis/README.md` §Citation; `git -C ~/workspace/Genesis describe --tags`
- `baselines/rl/rlpd_sac.py`, `baselines/rl/rlpd_pixel.py`, `baselines/rl/train_rlpd.py` (imports; DrQ/DrQ-v2 docstrings); `.venv-eval/bin/python -c "import stable_baselines3"` → 2.8.0
- Web: DLR-RM/stable-baselines3 `CITATION.bib`; proceedings.mlr.press v202/ball23a, v80/haarnoja18b, v164/mandlekar22a; facebookresearch/drqv2 README; denisyarats/drq README; nicklashansen/tdmpc2 README; NM512/r2dreamer README; kevinzakka/ibc README; arxiv.org/abs/2603.18202; api.crossref.org for 10.1177/02783649241273668, 10.1038/s41586-025-08744-2, 10.15607/RSS.2023.XIX.026; kinovarobotics.com/product/gen3-lite-robots; Springer 10.3758/PBR.16.2.225; dblp conf/iclr/YaratsFLP22
- `~/workspace/overleaf/6a96f0e5337340dfd4edea88/sample-base.bib`, `software.bib`, `main.tex`; BibTeX test of `@software` / duplicate keys in the scratchpad `toctest/t_sw.*`

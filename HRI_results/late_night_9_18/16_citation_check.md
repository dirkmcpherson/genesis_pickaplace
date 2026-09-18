# Item 16 — Every citation in the paper matched against the bibliography

**Status: done.** All `\cite*` keys in `main.tex`, `sections/*.tex` (excluding `04-evaluation-bk.tex`),
`frontmatter/*.tex`, `appendix/a-research-methods.tex`, `appendix/b-online-resources.tex`, `back/*.tex`, `tables/*.tex`
were extracted mechanically and compared with the keys defined in `sample-base.bib` (the only file `main.tex` loads) and
`software.bib` (not loaded; template junk only). **Four keys are cited but undefined**; BibTeX for each is in
`10_bibtex_missing.md` and repeated below. Six `\todo{cite…}` requests are answered with proposed keys. One discrepancy
between a sentence and the project's own data is flagged.

Reproduce the check:
```bash
cd ~/workspace/overleaf/6a96f0e5337340dfd4edea88
FILES="main.tex $(ls sections/*.tex | grep -v 04-evaluation-bk) frontmatter/*.tex appendix/a-research-methods.tex appendix/b-online-resources.tex back/*.tex tables/*.tex"
grep -ohE '\\cite(p|t|author|year|alp|alt|yearpar)?\*?(\[[^]]*\])*\{[^}]*\}' $FILES | sed -E 's/.*\{([^}]*)\}/\1/' | tr ',' '\n' | sed 's/^ *//;s/ *$//' | sort -u > /tmp/used
grep -oE '^@[A-Za-z]+\{[^,]+' sample-base.bib | sed 's/.*{//' | sort -u > /tmp/defined
comm -23 /tmp/used /tmp/defined      # cited, not defined
```

## 1. Keys cited (26 unique) and where

| key | files | in `sample-base.bib`? |
|---|---|---|
| chi2024diffusionpolicy | 01, 02, 03, 05 | yes (thin: no volume/pages/doi) |
| staley2024agent | 02, 03 | yes |
| florence2022implicit | 01, 02, 03 | yes |
| mandlekar2021matters | 01, 02 | yes (arXiv; CoRL version in item 10) |
| hafner2023mastering | 02, 03 | yes (arXiv; Nature 2025 version in item 10) |
| cadene2024lerobot | 03, appendix A | yes (6 of 16 authors) |
| **ball2023efficient** | 02, 03 | **NO** |
| **morihira2026r2d** | 03 | **NO** |
| **rafailov2021vmail** | 03 | **NO** — same paper as `rafailov2021visual` (cited in 02) |
| **rouder2009** | appendix A (`app:stats`) | **NO** |
| rafailov2021visual | 02 | yes |
| zhao2024aloha, zakka2021ibc, wu2023daydreamer, torchvision2016, swamy2022causal, orsini2021matters, luo2024precise, lee2024behavior, kuhar2023learning, hansen2022modem, Genesis, fang2025demonstration, dreamerv3torch2024, de2019causal | 01/03/04 | yes |

(01 = `sections/01-introduction.tex`, 02 = related work, 03 = method, 04 = evaluation, 05 = conclusion.)
`sections/06-todo.tex` and `sections/dataset_characterization.tex` exist but are not `\input` by `main.tex` and contain no
`\cite`; `appendix/c-llm-generated.tex` (not `\input`) contains none either.

## 2. Missing keys — BibTeX

```bibtex
%% verified: proceedings.mlr.press/v202/ball23a.html
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

%% verified: ~/workspace/r2dreamer/17677_R2_Dreamer_Redundancy_Re.pdf p.1, arXiv 2603.18202, NM512/r2dreamer README
@inproceedings{morihira2026r2d,
  title     = {{R2-Dreamer}: Redundancy-Reduced World Models without Decoders or Augmentation},
  author    = {Morihira, Naoki and Nahar, Amal and Bharadwaj, Kartik and Kato, Yasuhiro and Hayashi, Akinobu and Harada, Tatsuya},
  booktitle = {The Fourteenth International Conference on Learning Representations (ICLR)},
  year      = {2026},
  eprint    = {2603.18202},
  archivePrefix = {arXiv},
  url       = {https://openreview.net/forum?id=Je2QqXrcQq}
}

%% verified: Springer 10.3758/PBR.16.2.225
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

**`rafailov2021vmail`**: this is the VMAIL paper — *Visual Adversarial Imitation Learning using Variational Models*,
Rafailov, Yu, Rajeswaran, Finn, NeurIPS 34, 2021 — which the bib already holds as `rafailov2021visual` (fields correct:
NeurIPS 34, pp. 3016–3028, 2021). Do **not** add a second entry (the bibliography would list the paper twice); change the
key in `sections/03-method.tex` ("VMAIL is a model-based adversarial imitation learner~\citep{rafailov2021vmail}") to
`rafailov2021visual`. `\llm` note inserted there.

## 3. `\todo{cite …}` requests and proposed keys

| file:line | request | proposal | status |
|---|---|---|---|
| `sections/02-related-work.tex:6` | "cite OG BC paper" | `\citep{pomerleau1988alvinn, bain1995framework}` — both **already in the bib** (ALVINN, NeurIPS 1 1988; Bain & Sammut, *Machine Intelligence 15*, 1995, the paper that coined "behavioural cloning"). Note `bain1995framework` has a stray trailing period in its title field. | ready |
| `sections/02-related-work.tex:10` | "how and why do we noise? Dragan 2017 jankbot?" (Boltzmann-distribution human model) | `dragan2017robot` — Anca D. Dragan, *Robot Planning with Mathematical Models of Human State and Action*, arXiv:1705.04226, 2017 (verified on arXiv; it is the standard tutorial-style source for Boltzmann-rational human models). The origin of the Boltzmann/max-ent human model in IRL is Ziebart et al. 2008 (`ziebart2008maximum`, AAAI) and, in cognitive science, Baker, Saxe & Tenenbaum 2009 (`baker2009action`, *Cognition* 113(3):329–349). I could not identify what "jankbot" refers to. | **USER MUST CHECK** which Dragan paper was meant; Ziebart/Baker fields from memory |
| `sections/02-related-work.tex:10` | "cite those two papers that are somewhere in here and Hang's work" (non-Markovian human noise) | the two papers = `de2019causal` (*Causal confusion in imitation learning*, NeurIPS 2019) and `swamy2022causal` (*Causal imitation learning under temporally correlated noise*, ICML 2022) — both already cited in the introduction. "Hang's work" = Hang Yu (Tufts AABL): the demonstration paper with Hang as author is `fang2025demonstration` (*Demonstration sidetracks: categorizing systematic non-optimality in human demonstrations*, RO-MAN 2025, **already in the bib and cited in the intro**). Hang's other AABL papers (RO-MAN 2024 "How much progress did I make?", IROS 2023 "From thumbs up to 10 out of 10", HRI-LBR 2021 "Active feedback learning with rich feedback", CHARM RO-MAN 2025) are about human *feedback*, not demonstrations. | **USER MUST CHECK** — if a different Hang Yu paper on demonstration noise exists it is not on the AABL publications page |
| `sections/03-method.tex:50` | SAC `\todo{cite}` | `haarnoja2018soft` (PMLR 80:1861–1870, verified) | ready |
| `sections/03-method.tex:50` | "RLPD benefits from both successful and failed demonstrations \todo{is there a citation for this?}" | RLPD's own experiments use low-quality offline data (D4RL *random*/*medium-replay* locomotion, Adroit human data) — cite `ball2023efficient` and point at its D4RL results; the older precedent that demonstrations need not be successful for off-policy RL with a demo buffer is DDPGfD, `vecerik2017leveraging` (arXiv:1707.08817, verified) and Gao et al. 2018 *Reinforcement learning from imperfect demonstrations* (`gao2018reinforcement`, arXiv:1802.05313, from memory). **But see Discrepancy 1** — our own data do not support the sentence. | **USER MUST CHECK**; recommend softening the sentence |
| `sections/05-conclusion.tex:7` | "cite TDMPC2 and R2Dreamer" | `\citep{hansen2024tdmpc2, morihira2026r2d}` (both in item 10) | ready |

```bibtex
%% verified: arxiv.org/abs/1705.04226
@article{dragan2017robot,
  title   = {Robot Planning with Mathematical Models of Human State and Action},
  author  = {Dragan, Anca D.},
  journal = {arXiv preprint arXiv:1705.04226},
  year    = {2017}
}
%% from memory — USER MUST CHECK pages
@inproceedings{ziebart2008maximum,
  title     = {Maximum Entropy Inverse Reinforcement Learning},
  author    = {Ziebart, Brian D. and Maas, Andrew and Bagnell, J. Andrew and Dey, Anind K.},
  booktitle = {Proceedings of the 23rd AAAI Conference on Artificial Intelligence},
  pages     = {1433--1438},
  year      = {2008}
}
%% from memory — USER MUST CHECK
@article{baker2009action,
  title   = {Action understanding as inverse planning},
  author  = {Baker, Chris L. and Saxe, Rebecca and Tenenbaum, Joshua B.},
  journal = {Cognition},
  volume  = {113},
  number  = {3},
  pages   = {329--349},
  year    = {2009}
}
%% verified: arxiv.org/abs/1707.08817
@article{vecerik2017leveraging,
  title   = {Leveraging Demonstrations for Deep Reinforcement Learning on Robotics Problems with Sparse Rewards},
  author  = {Vecerik, Mel and Hester, Todd and Scholz, Jonathan and Wang, Fumin and Pietquin, Olivier and Piot, Bilal and Heess, Nicolas and Roth{\"o}rl, Thomas and Lampe, Thomas and Riedmiller, Martin},
  journal = {arXiv preprint arXiv:1707.08817},
  year    = {2017}
}
%% from memory — USER MUST CHECK
@article{gao2018reinforcement,
  title   = {Reinforcement Learning from Imperfect Demonstrations},
  author  = {Gao, Yang and Xu, Huazhe and Lin, Ji and Yu, Fisher and Levine, Sergey and Darrell, Trevor},
  journal = {arXiv preprint arXiv:1802.05313},
  year    = {2018}
}
```
(`haarnoja2018soft`, `hansen2024tdmpc2`, `morihira2026r2d`, `ball2023efficient` are in the item-10 block.)

## 4. Bibliography entries with broken or junk fields

`sample-base.bib` (89 entries, 22 cited):
- **`dreamerv3torch2024` is defined twice** → BibTeX `Repeated entry` error (reproduced). Delete one.
- `Gauss1857`, `Lagrange1788` — acmart template samples; delete. The comment line `% RLC main.bib Version 2024.1` after
  `Lagrange1788` is harmless.
- Exact duplicates of the same work under two keys (neither cited): `schulman2017proximal` / `DBLP:journals/corr/SchulmanWDRK17`;
  `octo_2023` (RSS 2024) / `mees2024octo` (workshop entry with **no `year`**). Delete the unused twin of each.
- `bain1995framework` title ends in a period ("A Framework for Behavioural Cloning.").
- `@software` entries `Genesis`, `zakka2021ibc`, `torchvision2016`: ACM-Reference-Format.bst has no `software` type; it
  prints the title as an italic booktitle and drops `url`/`version` (verified with BibTeX on a copy). Use the `@misc`
  forms in item 10.
- `yolo11_ultralytics` carries `orcid`/`license` fields (ignored; uncited).
- `pmlr-v229-mandlekar23b` carries a full `abstract` (ignored; uncited).
- `cadene2024lerobot` uses `howpublished = "…"` with double quotes — valid, but the URL contains no `%`, fine.

`software.bib` — **never loaded** (`main.tex`: `\bibliography{sample-base}` only). Its contents are the acmart
software-citation template (`delebecque:hal-02090402*` Scilab, `cgal*`, `parmap*`, `simplemapper*`, `ad-wood-2003`,
`gf-tag-sound-repo`) and use biblatex-only entry types (`@softwareversion`, `@softwaremodule`, `@codefragment`) that
ACM-Reference-Format.bst does not define (BibTeX would warn and fall back to `misc`). Delete the file before submission (or leave it unreferenced).

## Discrepancies

1. **"RLPD benefits from both successful and failed demonstrations"** (`sections/03-method.tex:50`) vs the project's own
   evidence: the only all-data test (pick phase, `dHv2all` = 66 successes + 24 no-pick recordings + 16 operator-labelled
   failures) found **no gain** — RLPD LAST rnd30 0.554 vs raw-success 0.600, Δ −0.046, p 0.567
   (`paper/MORNING_TABLE_2026-09-04.md:124-129`; memory note `fail-tapes-omitted-from-full-set.md`); and the full-task
   human set the paper reports on, `dHfull_all` (74), **excludes** the 16 failure trials
   (`HRI_results/DEMO_SETS_2026-09-11.md`, appendix `appx:real2sim` already says so). So in this study RLPD was not given
   failed demonstrations and, where it was, they did not help. Either cite the literature claim as such ("has been
   shown to …") or drop the sentence.
2. `rafailov2021vmail` (03-method) and `rafailov2021visual` (02-related-work) are the same paper under two keys.
3. `hafner2023mastering` is cited as the DreamerV3 reference while the peer-reviewed Nature 2025 version exists (item 10).
4. `\ref{appx:hyperparameters}` (`03-method.tex:29`) and `\label{appx:characterization_definitions}` used as a reference
   (`04-evaluation.tex:139`) are unresolved cross-references, not citations — listed in `09_appendix_toc.md`.

## \llm notes inserted

- `sections/02-related-work.tex`, anchor `\todo{cite OG BC paper}` — appended
  `\llm{Item 16: propose \citep{pomerleau1988alvinn, bain1995framework} (both already in sample-base.bib); see HRI_results/late_night_9_18/16_citation_check.md.}`
- `sections/02-related-work.tex`, anchor `\todo{how and why do we noise? Dragan 2017 jankbot?}` — appended
  `\llm{Item 16: Boltzmann-rational human model: propose \cite{dragan2017robot} (arXiv 1705.04226, verified) + origin \cite{ziebart2008maximum}; non-Markovian noise "two papers" = \cite{de2019causal, swamy2022causal} (already cited in the intro), Hang's work = \cite{fang2025demonstration} (already in bib); BibTeX + USER MUST CHECK flags in 16_citation_check.md.}`
- `sections/03-method.tex`, anchor `~\citep{rafailov2021vmail}` — appended
  `\llm{Item 16: key rafailov2021vmail is not in sample-base.bib; it is the same paper as rafailov2021visual (cited in Related Work) -- change the key rather than adding a duplicate entry.}`
- `sections/03-method.tex`, anchor `(like SAC\todo{cite})` — see item 10.
- `sections/03-method.tex`, anchor `\todo{is there a citation for this?}` — appended
  `\llm{Item 16: literature: \cite{ball2023efficient} (D4RL random/medium data), \cite{vecerik2017leveraging}; BUT our own all-data test showed no gain from failure tapes (RLPD 0.554 v 0.600, p 0.57, MORNING_TABLE_2026-09-04) and dHfull_all excludes the 16 failed trials, so soften or drop; details in 16_citation_check.md.}`
- `sections/05-conclusion.tex`, anchor `\todo{cite TDMPC2 and R2Dreamer}` — see item 10.

## Sources

- `~/workspace/overleaf/6a96f0e5337340dfd4edea88/{main.tex, sample-base.bib, software.bib}`;
  `sections/{01-introduction,02-related-work,03-method,04-evaluation,05-conclusion,06-todo,dataset_characterization}.tex`;
  `frontmatter/{abstract,ccs}.tex`; `appendix/{a-research-methods,b-online-resources,c-llm-generated}.tex`;
  `back/{acks,ethics}.tex`; `tables/*.tex` — grep commands in §0
- BibTeX behaviour checks (duplicate key, `@software`): scratchpad `toctest/t_sw.tex`, `sw.bib`, `t_sw.bbl`
- `paper/MORNING_TABLE_2026-09-04.md` (lines 124–129), `~/.claude/projects/-home-james-workspace-genesis-pickaplace/memory/fail-tapes-omitted-from-full-set.md`, `HRI_results/DEMO_SETS_2026-09-11.md`
- Web: proceedings.mlr.press (v202/ball23a, v80/haarnoja18b), Springer 10.3758/PBR.16.2.225, arxiv.org/abs/{2603.18202, 1705.04226, 1707.08817}, aabl.cs.tufts.edu/publications.html (Hang Yu's papers), NM512/r2dreamer README, nicklashansen/tdmpc2 README

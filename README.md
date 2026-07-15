# UAV Testing Competition

<p align="center">
  <img src="figures/uav1.gif" width="45%" height="45%"/>
  <img src="figures/uav2.gif" width="45%" height="45%"/>
</p>

Unmanned Aerial Vehicles (UAVs) equipped with onboard cameras and various sensors have already demonstrated the possibility of autonomous flights in real environments, leading to great interest in various application scenarios: crop monitoring, surveillance, medical and food delivery.

Over the years, support for UAV developers has increased with open-access projects for software and hardware, such as the autopilot support provided by [PX4](https://github.com/PX4/PX4-Autopilot) and [Ardupilot](https://github.com/ArduPilot/ardupilot).
However, despite the necessity of systematically testing such complex and automated systems to ensure their safe operation in real-world environments, there has been relatively limited investment in this direction so far.

The UAV Testing Competition organized jointly by the [International Conference on Software Testing, Verification and Validation (ICST)](https://conf.researchr.org/home/icst-2026) and [Search-Based and Fuzz Testing (SBFT) workshop](https://search-based-and-fuzz-testing.github.io/sbft26/) is an initiative designed to inspire and encourage the Software Testing Community to direct their attention toward UAVs as a rapidly emerging and crucial domain. The joint call is meant to help interested authors/participants reduce travel costs by selecting the most convenient and close venue.
  
## Competition Call

*The UAV Testing competition is back on for a 3<sup>rd</sup> Edition at SBFT@ICSE and at ICST 2026!*

### SBFT Deadline Extension

The submission deadline for [SBFT@ICSE](https://search-based-and-fuzz-testing.github.io/sbft26/) has been **extended** to 7.12.2025 (AoE).

The competition call, guidelines, and evaluation, are identical for the two calls.
You will have the option to choose where you want to participate and compete against the other competitors who applied for the same venue.

### Submission - SBFT@ICSE

The submission details for [SBFT@ICSE](https://search-based-and-fuzz-testing.github.io/sbft26/) participants are as follows:
- **Submission Deadline**: ~~30.11.2025~~ 07.11.2025 (AoE)
- **Notification**: 22.12.2025 (AoE)

### Submission - ICST

The submission details for [ICST](https://conf.researchr.org/home/icst-2026) participants are as follows:
- **Submission Deadline**: Mid or Late February 2026 (tentative)
- **Notification**: Mid March 2026 (tentative)

## Announcements

**New this year!**  
We are pleased to announce that the UAV Testing Competition will include *wind conditions* as part of the evaluation.
All submissions will be tested both in standard environments and under varying wind, making the competition closer to real-world scenarios.

<!-- _Want to test out the new wind feature?_ -->
<!-- We have included **two new case studies**, namely [Mission 4](./snippets/case_studies/mission4.yaml) and [Mission 5](./snippets/case_studies/mission5.yaml), which specifically incorporates wind conditions. -->

## Getting Started

To get started with your submission, please check out our [Wiki Pages](https://github.com/skhatiri/UAV-Testing-Competition/wiki/Home), particularly, the [Getting Started](https://github.com/skhatiri/UAV-Testing-Competition/wiki/Getting-Started) page.


## References

If you use this tool in your research, please cite the following papers:

- **Sajad Khatiri**, Sebastiano Panichella, and Paolo Tonella, "Simulation-based Testing of Unmanned Aerial Vehicles with Aerialist," *In 2024 International Conference on Software Engineering (ICSE)*. [Link](https://dl.acm.org/doi/10.1145/3639478.3640031).

  ````{code-block} bibtex
  @inproceedings{icse2024Aerialist,
    title={Simulation-based Testing of Unmanned Aerial Vehicles with Aerialist},
    author={Khatiri, Sajad and Panichella, Sebastiano and Tonella, Paolo},
    booktitle={International Conference on Software Engineering (ICSE)},
    year={2024},
  }
  ````

- **SBFT Tool competition report** 
  ````{code-block} bibtex
  @inproceedings{SBFT-UAV2026,
    author       = {Ramazan Erdem Uysal and Ali Javadi and Prakash Aryan and Aren Babikian and  Dmytro Humeniuk and  Sajad Mazraehkhatiri and  Sebastiano Panichella},
    title        = {{SBFT} Tool Competition 2026 – UAV Testing Track},
    booktitle    = {International Workshop on Search-Based and Fuzz Testing,
                    SBFT@ICSE 2026},
    year         = {2026}
  }
  ````

- **ICST Tool competition report** 
  ````{code-block} bibtex
  @inproceedings{ICST-UAV2026,
    author       = {Ramazan Erdem Uysal and Ali Javadi and Prakash Aryan and Aren Babikian and  Dmytro Humeniuk and  Sajad Mazraehkhatiri and  Sebastiano Panichella},
    title        = {{ICST} Tool Competition 2026 – UAV Testing Track},
    booktitle    = {International Conference on Software Testing, Verification and Validation (ICST)},
    year         = {2026}
  }
  ````

- **Sajad Khatiri**, Sebastiano Panichella, and Paolo Tonella, "Simulation-based Test Case Generation for Unmanned Aerial Vehicles in the Neighborhood of Real Flights," *In 2023 IEEE 16th International Conference on Software Testing, Verification and Validation (ICST)*. [Link](https://ieeexplore.ieee.org/document/10132225).

  ````{code-block} bibtex
  @inproceedings{khatiri2023simulation,
    title={Simulation-based test case generation for unmanned aerial vehicles in the neighborhood of real flights},
    author={Khatiri, Sajad and Panichella, Sebastiano and Tonella, Paolo},
    booktitle={2023 16th IEEE International Conference on Software Testing, Verification and Validation (ICST)},
    year={2023},
  }
  ````

## License

The software we developed is distributed under MIT license. See the [license](./LICENSE.md) file.

## Contacts

Please refer to the [FAQ page](https://github.com/skhatiri/UAV-Testing-Competition/wiki/Home) in the Wiki.

You may also refer to (and contribute to) the [Discussions Page](https://github.com/skhatiri/UAV-Testing-Competition/discussions), where you may find user-submitted questions and corresponding answers.

You can also contact us directly using email:

- Ramazan Erdem Uysal, University of Bern, ramazan.uysal@unibe.ch
- Ali Javadi, University of Bern, ali.javadi@unibe.ch, 
- Prakash Aryan, University of Bern, prakash.aryan@unibe.ch
- Aren Babikian, University of Toronto, aren.babikian@utoronto.ca
- Dmytro Humeniuk, Polytechnique Montréal, dmytro.humeniuk@polymtl.ca, 
- Sajad Mazraehkhatiri, University of Bern, sajad.mazraehkhatiri@unibe.ch
- Sebastiano Panichella, University of Bern, sebastiano.panichella@unibe.ch

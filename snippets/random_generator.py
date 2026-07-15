"""
Random search baseline for the experimental comparison with the (1+1)-ES.

Inherits everything from ESGenerator (legal space, adaptive sampling,
_evaluate, archive, checkpoint, delivery) except the search mechanism: every
candidate is sampled i.i.d, with no mutation, sigma adaptation, or targeted
restart. This isolates the evolutionary mechanism's contribution.

Obstacle count is sampled uniformly in [1,3], the same legal range the ES's
mutation can reach (add/remove obstacle) -- so the comparison isolates the
search STRATEGY (guided mutation vs pure i.i.d. sampling), not also the
search SPACE. With ADAPTIVE=0 it degrades to uniform random on the fixed box.
"""
import random

from es_generator import ESGenerator
from geometry import _non_overlapping_gene, _random_gene
from models import Individual


class RandomGenerator(ESGenerator):
    def _sample_individual(self) -> Individual:
        """i.i.d. sample: 1-3 obstacles (uniform count). Each extra obstacle
        is retried against overlap with the ones already picked (same
        _non_overlapping_gene helper the ES uses when it adds one) -- a
        single blind draw + discard-on-overlap would collapse almost every
        sample to 1 obstacle, since _random_gene's adaptive positions are
        concentrated within ROUTE_JITTER (a few meters) of the route."""
        n = random.randint(1, 3)
        genes = [_random_gene()]
        for _ in range(n - 1):
            extra = _non_overlapping_gene(genes)
            if extra is None:
                break
            genes.append(extra)
        return Individual(genes)

    def _make_child(self, parent: Individual, sigma: float) -> Individual:
        """Ignores parent and sigma: independent sample (pure random search)."""
        return self._sample_individual()

    def _restart_individual(self) -> Individual:
        """Targeted restart is part of the ES method: this always returns a
        pure sample, so the baseline stays random even under stagnation."""
        return self._sample_individual()

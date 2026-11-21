from types import SimpleNamespace

import pytest

from d3k_diagnostics.main import _RangingStatistics


def measurement(target_id: int, distance_m: float | None, *, valid: bool = True):
    return SimpleNamespace(target_id=target_id, distance_m=distance_m, valid=valid)


def test_aggregates_measurements_for_only_the_configured_peer():
    statistics = _RangingStatistics(peer_id=2)
    statistics.add_measurement(measurement(2, 4.0))
    statistics.add_measurement(measurement(2, 6.0))
    statistics.add_measurement(measurement(3, 100.0))

    summary = statistics.snapshot_and_reset()

    assert summary.sample_count == 2
    assert summary.mean_distance_m == pytest.approx(5.0)
    assert summary.stddev_distance_m == pytest.approx(1.0)
    assert summary.min_distance_m == pytest.approx(4.0)
    assert summary.max_distance_m == pytest.approx(6.0)
    assert not summary.has_errors


def test_counts_range_failures_only_for_the_configured_peer():
    statistics = _RangingStatistics(peer_id=2)
    statistics.add_failure(SimpleNamespace(target_id=3, status=1))
    statistics.add_failure(SimpleNamespace(target_id=2, status=1))
    statistics.add_failure(SimpleNamespace(target_id=2, status=4))

    summary = statistics.snapshot_and_reset()

    assert summary.failures_by_status == (("1", 1), ("4", 1))
    assert summary.has_errors


def test_counts_invalid_measurements_for_the_configured_peer():
    statistics = _RangingStatistics(peer_id=2)
    statistics.add_measurement(measurement(2, None, valid=False))
    statistics.add_measurement(measurement(3, None, valid=False))

    summary = statistics.snapshot_and_reset()

    assert summary.sample_count == 0
    assert summary.invalid_measurements == 1
    assert summary.has_errors


def test_snapshot_resets_statistics_for_the_next_poll_interval():
    statistics = _RangingStatistics(peer_id=2)
    statistics.add_measurement(measurement(2, 5.0))

    first = statistics.snapshot_and_reset()
    second = statistics.snapshot_and_reset()

    assert first.sample_count == 1
    assert second.sample_count == 0
    assert second.mean_distance_m is None

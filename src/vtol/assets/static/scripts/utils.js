export function isPlainObject(value) {
  return Boolean(value) && typeof value === 'object' && !Array.isArray(value);
}

export function finiteNumber(value) {
  return typeof value === 'number' && Number.isFinite(value) ? value : Number.NaN;
}

export function formatMetric(value, unit = '', digits = 2) {
  if (!Number.isFinite(finiteNumber(value))) {
    return '--';
  }

  return `${value.toFixed(digits)}${unit}`;
}

export function formatDegrees(value) {
  return formatMetric(value, ' deg', 1);
}

export function formatLatLon(value, axis) {
  if (!Number.isFinite(finiteNumber(value))) {
    return '--';
  }

  const positive = axis === 'lat' ? 'N' : 'E';
  const negative = axis === 'lat' ? 'S' : 'W';
  const suffix = value >= 0 ? positive : negative;
  return `${Math.abs(value).toFixed(5)} deg ${suffix}`;
}

export function formatAgeFromSeconds(timestampSeconds) {
  if (!Number.isFinite(finiteNumber(timestampSeconds)) || finiteNumber(timestampSeconds) <= 0) {
    return '--';
  }

  const ageSeconds = Math.max(0, Date.now() / 1000 - finiteNumber(timestampSeconds));
  if (ageSeconds < 1) {
    return 'now';
  }

  return `${ageSeconds.toFixed(1)} s ago`;
}

export function formatFreshnessFromMs(ageMs) {
  if (!Number.isFinite(ageMs) || ageMs <= 0) {
    return 'just now';
  }

  if (ageMs < 1000) {
    return `${Math.round(ageMs)} ms`;
  }

  return `${(ageMs / 1000).toFixed(1)} s`;
}

export function gpsFixLabel(value) {
  const fixType = finiteNumber(value);
  if (!Number.isFinite(fixType)) {
    return '--';
  }

  const labels = {
    0: 'NO GPS',
    1: 'NO FIX',
    2: '2D',
    3: '3D',
    4: 'DGPS',
    5: 'RTK FLOAT',
    6: 'RTK FIX',
  };

  return labels[fixType] ?? String(fixType);
}

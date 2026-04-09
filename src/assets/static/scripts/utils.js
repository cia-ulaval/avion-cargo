export function isPlainObject(value) {
  return Boolean(value) && typeof value === 'object' && !Array.isArray(value);
}

export function formatMetric(value, unit = '', digits = 2) {
  if (!Number.isFinite(Number(value))) {
    return '--';
  }

  return `${Number(value).toFixed(digits)}${unit}`;
}

export function formatDegrees(value) {
  return formatMetric(value, ' deg', 1);
}

export function formatLatLon(value, axis) {
  if (!Number.isFinite(Number(value))) {
    return '--';
  }

  const positive = axis === 'lat' ? 'N' : 'E';
  const negative = axis === 'lat' ? 'S' : 'W';
  const suffix = Number(value) >= 0 ? positive : negative;
  return `${Math.abs(Number(value)).toFixed(5)} deg ${suffix}`;
}

export function formatAgeFromSeconds(timestampSeconds) {
  if (!Number.isFinite(Number(timestampSeconds)) || Number(timestampSeconds) <= 0) {
    return '--';
  }

  const ageSeconds = Math.max(0, Date.now() / 1000 - Number(timestampSeconds));
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
  const fixType = Number(value);
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

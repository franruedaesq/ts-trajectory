import { describe, expect, it, vi } from 'vitest';
import { KinematicConstraintError, TrajectoryError, ValidationError } from '../errors';
import { TrajectoryBuilder } from '../TrajectoryBuilder';

const waypoints = [
  { time: 0, positions: [0, 0] },
  { time: 1, positions: [1, 2] },
  { time: 2, positions: [3, 4] },
];

describe('Error classes', () => {
  it('ValidationError is an instance of TrajectoryError and Error', () => {
    const err = new ValidationError('bad input');
    expect(err).toBeInstanceOf(ValidationError);
    expect(err).toBeInstanceOf(TrajectoryError);
    expect(err).toBeInstanceOf(Error);
    expect(err.name).toBe('ValidationError');
    expect(err.message).toBe('bad input');
  });

  it('KinematicConstraintError is an instance of TrajectoryError and Error', () => {
    const err = new KinematicConstraintError('limit exceeded', 2, 1);
    expect(err).toBeInstanceOf(KinematicConstraintError);
    expect(err).toBeInstanceOf(TrajectoryError);
    expect(err).toBeInstanceOf(Error);
    expect(err.name).toBe('KinematicConstraintError');
    expect(err.dimension).toBe(2);
    expect(err.segment).toBe(1);
  });

  it('TrajectoryBuilder throws ValidationError on bad input', () => {
    const builder = new TrajectoryBuilder();
    expect(() => builder.plan([{ time: 0, positions: [0] }], { interpolationType: 'linear' })).toThrow(
      ValidationError,
    );
  });

  it('TrajectoryBuilder throws KinematicConstraintError on velocity violation', () => {
    const builder = new TrajectoryBuilder();
    expect(() =>
      builder.plan(waypoints, { interpolationType: 'linear', maxVelocity: [0.001, 0.001] }),
    ).toThrow(KinematicConstraintError);
  });

  it('KinematicConstraintError has correct dimension and segment', () => {
    const builder = new TrajectoryBuilder();
    try {
      builder.plan(waypoints, { interpolationType: 'linear', maxVelocity: [100, 0.001] });
    } catch (err) {
      expect(err).toBeInstanceOf(KinematicConstraintError);
      const kcErr = err as KinematicConstraintError;
      expect(kcErr.dimension).toBe(1);
      expect(kcErr.segment).toBeGreaterThanOrEqual(0);
    }
  });
});

describe('TrajectoryBuilder observability hooks', () => {
  it('calls onPlanStart before planning', () => {
    const onPlanStart = vi.fn();
    const builder = new TrajectoryBuilder({ onPlanStart });
    builder.plan(waypoints, { interpolationType: 'linear' });
    expect(onPlanStart).toHaveBeenCalledOnce();
    expect(onPlanStart).toHaveBeenCalledWith(waypoints, { interpolationType: 'linear' });
  });

  it('calls onPlanComplete after successful planning', () => {
    const onPlanComplete = vi.fn();
    const builder = new TrajectoryBuilder({ onPlanComplete });
    const traj = builder.plan(waypoints, { interpolationType: 'cubic' });
    expect(onPlanComplete).toHaveBeenCalledOnce();
    const [trajArg, durationMs] = onPlanComplete.mock.calls[0];
    expect(trajArg).toBe(traj);
    expect(typeof durationMs).toBe('number');
    expect(durationMs).toBeGreaterThanOrEqual(0);
  });

  it('calls onPlanError when planning fails with a ValidationError', () => {
    const onPlanError = vi.fn();
    const builder = new TrajectoryBuilder({ onPlanError });
    expect(() =>
      builder.plan([{ time: 0, positions: [0] }], { interpolationType: 'linear' }),
    ).toThrow(ValidationError);
    expect(onPlanError).toHaveBeenCalledOnce();
    expect(onPlanError.mock.calls[0][0]).toBeInstanceOf(ValidationError);
  });

  it('calls onPlanError when a KinematicConstraintError is thrown, then re-throws', () => {
    const onPlanError = vi.fn();
    const builder = new TrajectoryBuilder({ onPlanError });
    expect(() =>
      builder.plan(waypoints, { interpolationType: 'linear', maxVelocity: [0.001, 0.001] }),
    ).toThrow(KinematicConstraintError);
    expect(onPlanError).toHaveBeenCalledOnce();
    expect(onPlanError.mock.calls[0][0]).toBeInstanceOf(KinematicConstraintError);
  });

  it('does not call onPlanComplete when planning fails', () => {
    const onPlanComplete = vi.fn();
    const builder = new TrajectoryBuilder({ onPlanComplete });
    expect(() =>
      builder.plan([{ time: 0, positions: [0] }], { interpolationType: 'linear' }),
    ).toThrow();
    expect(onPlanComplete).not.toHaveBeenCalled();
  });

  it('does not call onPlanError when planning succeeds', () => {
    const onPlanError = vi.fn();
    const builder = new TrajectoryBuilder({ onPlanError });
    builder.plan(waypoints, { interpolationType: 'linear' });
    expect(onPlanError).not.toHaveBeenCalled();
  });

  it('works fine with no options (default constructor)', () => {
    const builder = new TrajectoryBuilder();
    expect(() => builder.plan(waypoints, { interpolationType: 'linear' })).not.toThrow();
  });
});

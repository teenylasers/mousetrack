allow_error = 1e-4;
assert(abs(normpdf(0, 1, 0.2) - 0.391043) < allow_error)
assert(abs(normpdf([0; 2], [1 0; 0 3], [0.2; 0.3]) - 0.0556) < allow_error)
assert(abs(normpdf([0 2 1.2], [1 0 0.2; 0 3 0.1; 0.2 0.1 2], [0.12 1.3 1]) - 0.02373) < allow_error)

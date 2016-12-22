# Notes on Matlab version

## PnP solver

The current implementation assumes that EPnP provided by [EPFL](http://cvlab.epfl.ch/EPnP/index.php) is available as `efficient_php.m`.
The file commited in this repositroy is just a placeholder and is supposed to be replaced by the one downloaded from the official web site by the user separately.

Also please notice that `efficient_pnp_gauss.m` can provide a better solution than `efficient_pnp.m` in geenral, but it fails with our test dataset.
If you are sure that `efficient_pnp_gauss.m` works better for your data, you can use it instead of `efficient_pnp.m` by editing `tnm.m` as follows.

```
- [temp_R temp_T Cp{i,1}{1,1}] = efficient_pnp(Xp, q_h{i,1}, in_param);
+ [temp_R temp_T Cp{i,1}{1,1}] = efficient_pnp_gauss(Xp, q_h{i,1}, in_param);
```


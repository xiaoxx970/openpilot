from opendbc.car import structs


def get_effective_pcm_cruise(CP: structs.CarParams, has_longitudinal_control: bool) -> bool:
  # VW interfaces disable PCM cruise when Alpha Long is active, but cached
  # offroad CarParams can still describe the stock longitudinal configuration.
  if CP.brand == "volkswagen" and CP.alphaLongitudinalAvailable and has_longitudinal_control:
    return False
  return CP.pcmCruise

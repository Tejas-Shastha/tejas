import feeder_dqn_training as feeder

dqn = feeder.DQNSolver(20,5)
dqn.load_model()
tab, pol = dqn.getQTable()
# print(tab)
# print(pol)
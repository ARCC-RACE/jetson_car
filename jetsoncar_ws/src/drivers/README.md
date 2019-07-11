# Drivers

This stack includes all the various AI drivers that have been developed for the jetson car.

### nn_tools
A package that contains basic neural network (nn) data collection and preview tools.

## rosey
The first and most basic AI [(end-to-end CNN)](https://arxiv.org/abs/1604.07316) that takes in a single entire RGB image and determines how to steer.

### Performance
untested around the Jesuit Track

## rosey2
A basic AI [(end-to-end CNN)](https://arxiv.org/abs/1604.07316) that takes in a single RGB image cropped specifically FOV of the track and determines how to steer. The code in this AI package includes several improvements over rosey including the ability to put in a regularizer and to be able to merge multiple datasets easily.

### Performance
95% around the Jesuit Track

## rosey3
A more complex AI [(end-to-end CNN)](https://arxiv.org/abs/1604.07316) that gets temporal information by receiving 3 cropped RGB images instead of 1. Otherwise the same as rosey2.

### Performance
untested around the Jesuit Track

## hal
A complex Reinforcement Learning (RL) agent that learns through practice in simulated environments. Takes advantage of OpenAI Gym, Proximal Policy Optimization (PPO), and domain randomization.

### Performance
untested around the Jesuit Track

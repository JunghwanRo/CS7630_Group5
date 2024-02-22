<Predict>



<update ar>

# Calculate expected measurement Z_hat based on current state and landmark position L.
Z_hat = ...

# Calculate measurement residual.
y = Z - Z_hat

# Calculate measurement matrix H.
H = ...

# Calculate Kalman gain K.
S = H * self.P * H.T + R
K = self.P * H.T * inv(S)

# Update state estimate and covariance matrix.
self.X = self.X + K * y
self.P = (I - K * H) * self.P


<update compass>

# Calculate expected measurement Z_hat based on current state and landmark position L.
Z_hat = ...

# Calculate measurement residual.
y = Z - Z_hat

# Calculate measurement matrix H.
H = ...

# Calculate Kalman gain K.
S = H * self.P * H.T + R
K = self.P * H.T * inv(S)

# Update state estimate and covariance matrix.
self.X = self.X + K * y
self.P = (I - K * H) * self.P

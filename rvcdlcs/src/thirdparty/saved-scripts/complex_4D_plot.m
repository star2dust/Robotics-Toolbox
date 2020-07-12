close all
clear

r = (0:0.025:1)';                        % create a matrix of complex inputs
theta = pi*(-1:0.05:1);
z = r*exp(1i*theta);
w = z.^3;                                % calculate the complex outputs

surf(real(z),imag(z),real(w),imag(w))    % visualize the complex function using surf
xlabel('Real(z)')
ylabel('Imag(z)')
zlabel('Real(w)')
cb = colorbar;
cb.Label.String = 'Imag(w)';
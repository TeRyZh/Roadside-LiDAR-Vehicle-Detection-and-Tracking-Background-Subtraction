function [bg_img, background_mode] = background_dmd(img)

if size(img, 3) == 3 
    img = rgb2gray(img);
end
img = im2double(img);

X = img(:,1:end-1);
Xprime = img(:,2:end);

%%
r=1;
[U,Sigma,V] = svd(X,'econ');
Sigmar = Sigma(1:r, 1:r);
Ur = U(:, 1:r);
Vr = V(:, 1:r);
Atilde = Ur'*Xprime*Vr*diag(1./diag(Sigmar)); 
[W, Lambda] = eig(Atilde);
mu = diag(Lambda);
omega = log(mu);
Phi = Xprime*Vr/Sigmar*W;

y0 = Phi\img(:,1);
v_modes = zeros(r,length(X(1,:)));
for i = 1:length(X(1,:))
    v_modes(:,i) = (y0.*exp(omega*i));
end
v_dmd = Phi*v_modes;
v_dmd = abs(v_dmd);
% v_sparse = X - v_dmd;
% 
% residual_matrix = v_sparse.*(v_sparse < 0);
% v_dmd = residual_matrix + abs(v_dmd);

%%
background_mode = v_dmd(:,1);
bg_img = ones([size(v_dmd,1),size(v_dmd,2)+1]);

for i = 1 : size(bg_img,2)
    bg_img(:,i) = background_mode;
end

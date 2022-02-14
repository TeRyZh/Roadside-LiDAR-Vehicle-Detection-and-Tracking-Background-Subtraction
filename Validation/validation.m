%% point level validate

frame_samples = [200, 400, 600, 800, 1000];

ranges = [30, 100];

model_precisions = [0,0];
model_recalls = [0,0];
model_F1s = [0,0];

comparable_precisions = [0, 0];
comparable_recalls = [0, 0];
comparable_F1s = [0, 0];

for i = 1 : max(size(frame_samples))

    frame_num = frame_samples(i);

    ground_truth_file = sprintf('2021-10-20-15-30-35_frame_%d.ply', frame_num);
    model_file = sprintf('CFTA_frame_%d.ply', frame_num);
    comparable_file = sprintf('ReferenceModel_frame_%d.ply', frame_num);

    ground_truth_ptCloud = pcread(ground_truth_file);
    model_ptCloud = pcread(model_file);
    comparable_ptCloud = pcread(comparable_file);

    ground_truth_ptCloud_XYZ = ground_truth_ptCloud.Location;
    model_ptCloud_XYZ = model_ptCloud.Location;
    comparable_ptCloud_XYZ = comparable_ptCloud.Location;

    [~, ~, grouond_ranges] = cart2sph(ground_truth_ptCloud_XYZ(:,1), ground_truth_ptCloud_XYZ(:,2), ground_truth_ptCloud_XYZ(:,3));
    [~, ~, model_ranges] = cart2sph(model_ptCloud_XYZ(:,1), model_ptCloud_XYZ(:,2), model_ptCloud_XYZ(:,3));
    [~, ~, compare_ranges] = cart2sph(comparable_ptCloud_XYZ(:,1), comparable_ptCloud_XYZ(:,2), comparable_ptCloud_XYZ(:,3));

    for j = 1 : 2

        if j == 1

            begin = 0;
        
        else

            begin = ranges(1);

        end
        
        range_div = ranges(j);
        
        filtered_ground_XYZ = ground_truth_ptCloud_XYZ((grouond_ranges > begin) & (grouond_ranges <= range_div),:);
        filtered_model_XYZ = model_ptCloud_XYZ((model_ranges > begin) & (model_ranges <= range_div),:);
        filtered_comparable_XYZ = comparable_ptCloud_XYZ((compare_ranges > begin) & (compare_ranges <= range_div),:);
        
        % calculate model performance metrics
        model_accurate_det = ismember(filtered_model_XYZ, filtered_ground_XYZ,'rows');
        model_false_det = 1 - model_accurate_det;
        mdoel_missed = 1- ismember(filtered_ground_XYZ, filtered_model_XYZ,'rows');

        model_tp = sum(model_accurate_det == 1);
        model_fp = sum((model_false_det == 1));
        model_fn = sum((mdoel_missed == 1));

        model_precisions(j) = model_tp / (model_tp + model_fp);
        model_recalls(j) = model_tp / (model_tp + model_fn);
        model_F1s(j) = (2 * model_precisions(j) * model_recalls(j)) / (model_precisions(j) + model_recalls(j));

        % calculate comparable model performance metrics
        compare_accurate_det = ismember(filtered_comparable_XYZ, filtered_ground_XYZ,'rows');
        compare_false_det = 1 - compare_accurate_det;
        compare_mdoel_missed = 1- ismember(filtered_ground_XYZ, filtered_comparable_XYZ,'rows');

        compare_tp = sum(compare_accurate_det == 1);
        compare_fp = sum((compare_false_det == 1));
        compare_fn = sum((compare_mdoel_missed == 1));

        comparable_precisions(j) = compare_tp / (compare_tp + compare_fp);
        comparable_recalls(j) = compare_tp / (compare_tp + compare_fn);
        comparable_F1s(j) = (2 * comparable_precisions(j) * comparable_recalls(j)) / (comparable_precisions(j) + comparable_recalls(j));

    end

end
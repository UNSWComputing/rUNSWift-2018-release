'''
Training program for Gaussian Mixture Models
Required packages: 
    sklearn: pip install scikit-learn
    opencv: apt-get install python-opencv
Run this program with -h flag to see all possible argument options
'''

import cv2
import numpy as np
import sys
import os
import time
import struct
from sklearn.mixture import GaussianMixture
from sklearn.naive_bayes import BaseNB
from sklearn.model_selection import train_test_split, GridSearchCV, RandomizedSearchCV
from sklearn.decomposition import PCA
from sklearn.pipeline import Pipeline
from sklearn.externals import joblib

INPUT_X = 28
INPUT_Y = 28

def loadImage(path):
        imgs = []
        labels = []
        for file in os.listdir(path):
                if file.startswith('.'):
                    continue
                file_path = os.path.join(path, file)
                temp = cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)
                temp = cv2.resize(temp, (INPUT_X, INPUT_Y))
                temp = np.reshape(temp, (1, -1))[0]
                if file.startswith('t') or file.startswith('T'):
                        imgs.append(temp)
                        labels.append([1, 0])
                elif file.startswith('f') or file.startswith('F'):
                        imgs.append(temp)
                        labels.append([0, 1])
        return imgs, labels


def preproccess(path):
        X, Y = loadImage(path)
        X_train, X_test, y_train, y_test = train_test_split(X, Y, test_size=0.01, random_state=0)
        X_train = np.reshape(X_train, [-1, INPUT_X, INPUT_Y, 1])
        X_train = np.concatenate([X_train, np.flip(X_train, 1), np.flip(X_train, 2), np.flip(np.flip(X_train, 1), 2)])
        X_train = np.reshape(X_train, [-1, INPUT_X * INPUT_Y])
        y_train = np.concatenate([y_train, y_train, y_train, y_train])
        X_test = np.array(X_test)
        y_test = np.array(y_test)

        return X_train, X_test, y_train, y_test
    

class ClassCondGMM(BaseNB):
    """
    Class Conditional Gaussian Mixture Model
    """
    def __init__(self, n_components, covariance_type, n_init):
        self.n_components = n_components
        self.covariance_type = covariance_type
        self.n_init = n_init

    def fit(self, X, y):
        self.classes_ = np.arange(2, dtype=int)
        self.class_prior_ = y.mean(axis=0) # TODO: find a way to estimate the true prior
        class_cond_models = []

        for i in range(len(self.classes_)):
            this_idx = y[:, i]==1
            this_x = X[this_idx, :]
            this_y = y[this_idx, :]
            this_model = GaussianMixture(
                n_components=self.n_components,
                covariance_type=self.covariance_type,
                n_init=self.n_init,
            )
            this_model.fit(this_x, this_y)
            class_cond_models.append(this_model)

        self.class_cond_models = class_cond_models
        return self

    def _joint_log_likelihood(self, X):
        joint_log_likelihood = []
        for i in range(np.size(self.classes_)):
            jointi = np.log(self.class_prior_[i])
            l_ij = self.class_cond_models[i].score_samples(X)
            joint_log_likelihood.append(jointi + l_ij)

        joint_log_likelihood = np.array(joint_log_likelihood).T
        return joint_log_likelihood

    def predict(self, X):
        multiclass_pred = super(ClassCondGMM, self).predict(X)
        one_hot_pred = np.zeros((X.shape[0], np.size(self.classes_)))
        one_hot_pred[np.arange(X.shape[0]), multiclass_pred.ravel()] = 1
        return one_hot_pred

def train_model(xtrain, ytrain, n_pca_components, n_mixture_components, covariance_type='full', n_init=1):
    """
    Train the model hyper-parameters by cross validation.
    """
    pca = PCA(
        svd_solver='randomized',
        n_components=10)
    ccgmm = ClassCondGMM(
        covariance_type=covariance_type,
        n_init=n_init,
        n_components=10
    )
    pipe = Pipeline(steps=[('pca', pca), ('ccgmm', ccgmm)])

    estimator = GridSearchCV(
        pipe,
        cv=5,
        param_grid=dict(
            pca__n_components=n_pca_components,
            ccgmm__n_components=n_mixture_components
        ),
        verbose=5,
        n_jobs=1,
        pre_dispatch=2,
        scoring='accuracy'
    )
    estimator.fit(xtrain, ytrain)

    joblib.dump(
        estimator.best_estimator_,
        'model.pkl',
    )
    
    return estimator


def make_predictions(X_test, y_test):
    model = joblib.load('model.pkl')

    t1 = time.time()
    class_pred = model.predict(X_test)
    t2 = time.time()
    class_logprob = model.predict_log_proba(X_test)

    accuracy, loglike = predictive_performance(X_test, y_test, class_pred, class_logprob)
    print('Average test accuracy=' + str(accuracy))
    print('Average test likelihood=' + str(loglike))
    print('time used', t2 - t1)


def make_special_test(path):
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE).T
        X = cv2.resize(img, (INPUT_X, INPUT_Y))
        X = np.reshape(X, (1, -1))[0]
        
        model = joblib.load("model.pkl")
        X_ = model.named_steps['pca'].transform([X])
        print('PCA tranformed')
        print('--------------')
        print(X_)     
        print('--------------')
        prob = model.named_steps['ccgmm']._joint_log_likelihood(X_)
        print('log probability')
        print(prob)
        print('predict')
        print(model.predict([X]))


def predictive_performance(xdata, ydata, class_pred, class_logprob):
    correct = np.zeros(xdata.shape[0])
    ltest = np.zeros(xdata.shape[0])
    for i, x in enumerate(xdata):
        correct[i] = np.all(ydata[i, :] == class_pred[i,:])
        ltest[i] = class_logprob[i, np.argmax(ydata[i,:])]

        if correct[i] == 0:
            # print failed cases
            print(np.exp(class_logprob[i]))
            
    accuracy = correct.mean()
    loglike = ltest.mean()
    return accuracy, loglike


def nao_dump_model(dump_name):
    # dump models for nao
    model = joblib.load('model.pkl')
    pca, ccgmm = model.named_steps['pca'], model.named_steps['ccgmm']

    import os
    os.system('rm ./means/*')
    for i, m in enumerate(ccgmm.class_cond_models[0].means_):
        rec = np.reshape(pca.inverse_transform(m), (INPUT_X, INPUT_Y))
        cv2.imwrite('means/T_'+'_'+str(ccgmm.class_cond_models[0].weights_[i])+'_'+str(i)+'.png', rec)
    for i, m in enumerate(ccgmm.class_cond_models[1].means_):
        rec = np.reshape(pca.inverse_transform(m), (INPUT_X, INPUT_Y))
        cv2.imwrite('means/F_'+'_'+str(ccgmm.class_cond_models[1].weights_[i])+'_'+str(i)+'.png', rec)

    with open('{}.pca'.format(dump_name), 'wb') as file:
        # dump pca
        n_features, n_components = INPUT_X * INPUT_Y, pca.n_components_
        print('PCA size: ', n_components)
        file.write(struct.pack('iii', INPUT_X, INPUT_Y, n_components))
        means, components = pca.mean_, pca.components_.T
        ## pca means
        for i in range(n_features):
            file.write(struct.pack('d', means[i]))
        ## pca trasform matrix
        for i in range(n_features):
            for j in range(n_components):
                file.write(struct.pack('d', components[i, j]))
    with open('{}.gmm'.format(dump_name), 'wb') as file:
        # dump gmm
        n_features, n_components, n_classes = pca.n_components_, ccgmm.n_components, ccgmm.classes_.shape[0]
        print('GMM size: ', n_components)
        file.write(struct.pack('iii', n_features, n_components, n_classes))
        ## dump log prior
        prior = ccgmm.class_prior_
        for i in range(n_classes):
            file.write(struct.pack('d', np.log(prior[i])))
        for cls in range(n_classes):
            gmm = ccgmm.class_cond_models[cls]
            weights, means, covariances, precisions = gmm.weights_, gmm.means_, gmm.covariances_, gmm.precisions_
            ## gmm log weights
            for i in range(n_components):
                file.write(struct.pack('d', np.log(weights[i])))
                ## gmm means
                for j in range(n_features):
                    file.write(struct.pack('d', means[i, j]))
                ## log normalised factor of gaussian distribution
                chol = np.linalg.cholesky(covariances[i])
                # cholesky decomposition
                # cv = chol * chol.T
                # det(cv) = det(chol) * det(chol.T) = det(chol) ** 2 = prod(diag(chol)) ** 2
                # log(det(cv)) = 2 * sum(log(diag(chol)))
                log_det = 2 * np.sum(np.log(np.diagonal(chol)))
                file.write(struct.pack('d', - 0.5 * n_features * np.log(2 * np.pi) - 0.5 * log_det))
                ## gmm precision matrix
                for j in range(n_features):
                    for k in range(n_features):
                        file.write(struct.pack('d', precisions[i, j, k]))


if __name__ == '__main__':
    from argparse import ArgumentParser
    parser = ArgumentParser()
    parser.add_argument('--train', help='Train the model using the data stored in DIR, and save it as model.pkl in the current directory', metavar='DIR')
    parser.add_argument('--pca_min', help='The minimum number of PCA components', type=int, default=20, metavar='INT')
    parser.add_argument('--gmm_min', help='The minimum number of GMM components', type=int, default=10, metavar='INT')
    parser.add_argument('--pca_max', help='The maximum number of PCA components', type=int, default=80, metavar='INT')
    parser.add_argument('--gmm_max', help='The maximum number of GMM components', type=int, default=50, metavar='INT')
    parser.add_argument('--pca_step', help='The number of steps in PCA components for grid search', type=int, default=5, metavar='INT')
    parser.add_argument('--gmm_step', help='The number of steps in GMM components for grid search', type=int, default=5, metavar='INT')
    parser.add_argument('--test', help='Display the trained model predicted class and probability of the given sample in FILE', metavar='FILE')
    parser.add_argument('--dump', help='Dump the trained model to FILE.pca and FILE.gmm', metavar='FILE')

    args = parser.parse_args()
    print(args)
    if args.train is not None:
        # pre-processing
        X_train, X_test, y_train, y_test = preproccess(args.train)
        # training
        n_pca_components = np.arange(args.pca_min, args.pca_max, args.pca_step)
        n_mixture_components = np.arange(args.gmm_min, args.gmm_max, args.gmm_step)
        model = train_model(X_train, y_train, n_pca_components, n_mixture_components)
        # model = joblib.load("model.pkl")
        print("Training accuracy:", model.score(X_train, y_train))
        # prediction and evaluation
        make_predictions(X_test, y_test)
    if args.dump is not None:
        # dump 
        nao_dump_model(args.dump)
    if args.test is not None:
        # testing
        make_special_test(args.test)